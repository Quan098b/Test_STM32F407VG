#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Thu_vien_L298.h"
#include "Motor_Encoder_Cua_Quan.h"

#define BAUDRATE 115200U

/* ================== CHỈNH TẠI ĐÂY ================== */
#define ENCODER_PPR        600L
#define ENCODER_QUAD_MULT  4L
#define COUNTS_PER_REV     (ENCODER_PPR * ENCODER_QUAD_MULT)

/* PWM mềm trên ENA=PC8 */
#define PWM_PERIOD_MS      5U      /* 5ms ~ 200Hz */
#define DUTY_PERCENT       45U     /* giảm tốc: 30..60 là thường hợp lý */

/* In trạng thái */
#define PRINT_PERIOD_MS    500U

/* Nếu đang chạy mà 2s không có xung -> dừng + reset */
#define NO_MOVE_RESET_MS   2000U
/* =================================================== */

/* ================== SysTick 1ms ================== */
static volatile uint32_t g_ms = 0;
void SysTick_Handler(void) { g_ms++; }
static inline uint32_t millis_ms(void) { return g_ms; }

/* ================== USART2 TX/RX PA2/PA3 ================== */
static uint32_t get_pclk1_hz(void)
{
    SystemCoreClockUpdate();
    static const uint8_t apb_tbl[8] = {1,1,1,1,2,4,8,16};
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;
    return SystemCoreClock / apb_tbl[ppre1];
}

static void usart2_init_tx_rx(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 TX, PA3 RX: AF7 */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));
    GPIOA->MODER &= ~(3U << (3U * 2U));
    GPIOA->MODER |=  (2U << (3U * 2U));

    GPIOA->AFR[0] &= ~(0xFU << (2U * 4U));
    GPIOA->AFR[0] |=  (7U   << (2U * 4U));
    GPIOA->AFR[0] &= ~(0xFU << (3U * 4U));
    GPIOA->AFR[0] |=  (7U   << (3U * 4U));

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->BRR = (uint16_t)(get_pclk1_hz() / BAUDRATE);

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;
}

static void usart2_write_byte(uint8_t b)
{
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') usart2_write_byte('\r');
        usart2_write_byte((uint8_t)ptr[i]);
    }
    return len;
}

/* non-block RX */
static int usart2_read_byte_nonblock(void)
{
    if (USART2->SR & USART_SR_RXNE) return (int)(uint8_t)USART2->DR;
    return -1;
}

/* ================== Command line parser ================== */
#define CMD_BUF_SZ 32
static char g_cmd_buf[CMD_BUF_SZ];
static uint8_t g_cmd_len = 0;

/* return 1 when a line is ready */
static int cmd_poll_line(char *out, uint32_t out_sz)
{
    int ch;
    while ((ch = usart2_read_byte_nonblock()) >= 0)
    {
        if (ch == '\r' || ch == '\n')
        {
            if (g_cmd_len == 0) return 0;
            g_cmd_buf[g_cmd_len] = '\0';
            strncpy(out, g_cmd_buf, out_sz - 1U);
            out[out_sz - 1U] = '\0';
            g_cmd_len = 0;
            return 1;
        }
        else
        {
            if (g_cmd_len < (CMD_BUF_SZ - 1U)) {
                g_cmd_buf[g_cmd_len++] = (char)ch;
            }
        }
    }
    return 0;
}

/* ================== Global handles (libraries) ================== */
static EncoderEXTI_CQ_Handle g_enc;
static L298_Handle_t g_mot;

/* EXTI group 5..9 */
void EXTI9_5_IRQHandler(void)
{
    EncoderEXTI_CQ_IRQHandler(&g_enc);
}

/* ================== Main logic ================== */
int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    usart2_init_tx_rx();

    /* Encoder: C1=PC7, C2=PC9 */
    g_enc.gpioA = GPIOC; g_enc.pinA = 7;  /* A */
    g_enc.gpioB = GPIOC; g_enc.pinB = 9;  /* B */
    EncoderEXTI_CQ_Init(&g_enc);

    /* L298: ENA=PC8, IN1=PC13, IN2=PE5 */
    L298_Pins_t pins = {
        .ENA_Port = GPIOC, .ENA_Pin = 8,
        .IN1_Port = GPIOC, .IN1_Pin = 13,
        .IN2_Port = GPIOE, .IN2_Pin = 5
    };
    L298_Init_SoftPWM(&g_mot, &pins, PWM_PERIOD_MS);
    L298_SetDutyPercent(&g_mot, DUTY_PERCENT);
    L298_Coast(&g_mot);
    L298_Enable(&g_mot, 0);

    printf("START: L298 + Encoder (library mode)\n");
    printf("COUNTS_PER_REV=%ld | softPWM=%ums | duty=%u%%\n",
           (long)COUNTS_PER_REV,
           (unsigned)PWM_PERIOD_MS,
           (unsigned)DUTY_PERCENT);
    printf("Nhap so vong: vd 3, -2, 0. (RST de reset encoder)\n");

    /* control state */
    uint8_t  running = 0;
    int32_t  target_counts = 0;

    uint32_t last_print = millis_ms();
    uint32_t last_move  = millis_ms();
    int32_t  last_cnt   = EncoderEXTI_CQ_GetCount(&g_enc);

    char line[CMD_BUF_SZ];

    while (1)
    {
        uint32_t now = millis_ms();

        /* (1) Update motor soft PWM (library) */
        L298_SoftPWM_Update(&g_mot, now);

        /* (2) Receive command */
        if (cmd_poll_line(line, sizeof(line)))
        {
            if (strcmp(line, "RST") == 0 || strcmp(line, "rst") == 0)
            {
                EncoderEXTI_CQ_Reset(&g_enc);
                printf("OK: encoder reset -> 0\n");
            }
            else
            {
                long rev_cmd = strtol(line, NULL, 10);

                if (rev_cmd == 0)
                {
                    running = 0;
                    L298_Enable(&g_mot, 0);
                    L298_Coast(&g_mot);
                    printf("STOP\n");
                }
                else
                {
                    long rev_abs = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
                    target_counts = (int32_t)(rev_abs * (long)COUNTS_PER_REV);

                    EncoderEXTI_CQ_Reset(&g_enc);
                    last_cnt  = 0;
                    last_move = now;

                    if (rev_cmd > 0) L298_SetDir(&g_mot, L298_DIR_FWD);
                    else             L298_SetDir(&g_mot, L298_DIR_REV);

                    L298_Enable(&g_mot, 1);
                    running = 1;

                    printf("GO: rev=%ld -> target_counts=%ld\n",
                           rev_cmd, (long)target_counts);
                }
            }
        }

        /* (3) Control loop while running */
        if (running)
        {
            int32_t cnt = EncoderEXTI_CQ_GetCount(&g_enc);
            int32_t mag = (cnt < 0) ? -cnt : cnt;  /* chống “âm vòng” */

            /* detect movement / stall */
            if (cnt != last_cnt) {
                last_cnt = cnt;
                last_move = now;
            } else {
                if ((now - last_move) >= NO_MOVE_RESET_MS) {
                    running = 0;
                    L298_Enable(&g_mot, 0);
                    L298_Coast(&g_mot);
                    EncoderEXTI_CQ_Reset(&g_enc);
                    printf("WARN: %ums khong thay xung -> reset 0 va dung\n",
                           (unsigned)NO_MOVE_RESET_MS);
                }
            }

            /* reached target */
            if (running && (mag >= target_counts))
            {
                running = 0;
                L298_Enable(&g_mot, 0);
                L298_Coast(&g_mot);

                float rev_done = (float)mag / (float)COUNTS_PER_REV;
                printf("DONE: cnt=%ld (~%.3f vong)\n", (long)mag, rev_done);
            }
        }

        /* (4) Periodic print */
        if ((now - last_print) >= PRINT_PERIOD_MS)
        {
            last_print = now;

            int32_t c = EncoderEXTI_CQ_GetCount(&g_enc);
            int32_t c_abs = (c < 0) ? -c : c;

            float rev = (float)c_abs / (float)COUNTS_PER_REV;
            int32_t turns = c_abs / (int32_t)COUNTS_PER_REV;
            int32_t rem   = c_abs % (int32_t)COUNTS_PER_REV;

            printf("run=%u | cnt=%ld (abs=%ld) | rev=%.3f | %ld vong + %ld/%ld\n",
                   (unsigned)running,
                   (long)c, (long)c_abs,
                   rev,
                   (long)turns, (long)rem, (long)COUNTS_PER_REV);
        }
    }
}
