#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Thu_vien_L298.h"
#include "Motor_Encoder_Cua_Quan.h"   /* thư viện encoder EXTI của bạn */

/* ================= UART ================= */
#define BAUDRATE 115200U

/* ================== ENCODER ==================
   Bạn đang dùng: ENCODER_PPR=600, x4 => 2400 counts/vòng.
   Với GA37, thông số này rất dễ KHÔNG đúng theo trục ra.
   Hãy hiệu chuẩn: quay đúng 1 vòng trục ra -> đọc cnt tăng bao nhiêu -> đặt COUNTS_PER_REV theo thực tế.
*/
#define ENCODER_PPR        600L
#define ENCODER_QUAD_MULT  4L
#define COUNTS_PER_REV     (ENCODER_PPR * ENCODER_QUAD_MULT)

/* ================== L298 Soft PWM ==================
   PWM mềm chỉ để giảm tốc cơ bản (không mượt như timer PWM).
   period 2..10ms (100..500Hz).
*/
#define PWM_PERIOD_MS      5U

/* Điều khiển chống vượt: 2 pha FAST -> SLOW + BRAKE */
#define SPEED_FAST_8BIT    120U     /* 0..255 */
#define SPEED_SLOW_8BIT    35U      /* 0..255: tốc độ tiến gần */
#define SLOW_BAND_COUNTS   (COUNTS_PER_REV / 3)  /* còn ~0.33 vòng thì giảm tốc */
#define BRAKE_MS           200U     /* thắng điện */

/* In trạng thái */
#define PRINT_PERIOD_MS    500U

/* Nếu đang chạy mà 2s không có xung -> reset encoder, dừng */
#define NO_MOVE_RESET_MS   2000U

/* ================== SysTick ms ================== */
static volatile uint32_t g_ms = 0;
void SysTick_Handler(void) { g_ms++; }
static inline uint32_t millis_ms(void) { return g_ms; }

/* ================= USART2 TX/RX (PA2/PA3) ================= */
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
    while (!(USART2->SR & USART_SR_TXE)) { }
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

static int usart2_read_byte_nonblock(void)
{
    if (USART2->SR & USART_SR_RXNE) {
        return (int)(uint8_t)USART2->DR;
    }
    return -1;
}

/* ================== Command line parser ================== */
#define CMD_BUF_SZ 32
static char g_cmd_buf[CMD_BUF_SZ];
static uint8_t g_cmd_len = 0;

/* return 1 if received a full line */
static int cmd_poll_line(char *out, uint32_t out_sz)
{
    int ch;
    while ((ch = usart2_read_byte_nonblock()) >= 0)
    {
        if (ch == '\r' || ch == '\n')
        {
            if (g_cmd_len == 0) return 0; /* ignore empty line */
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

/* ================== Encoder library binding ==================
   Bạn dùng encoder EXTI thư viện Motor_Encoder_Cua_Quan.
   PC7 + PC9 nằm trong EXTI9_5_IRQHandler.
*/
static EncoderEXTI_CQ_Handle g_enc;

void EXTI9_5_IRQHandler(void)
{
    EncoderEXTI_CQ_IRQHandler(&g_enc);
}

/* ================== Helpers ================== */
static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

/* ================== Main ================== */
int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    usart2_init_tx_rx();

    /* Encoder: C1=PC7, C2=PC9 */
    g_enc.gpioA = GPIOC; g_enc.pinA = 7;
    g_enc.gpioB = GPIOC; g_enc.pinB = 9;
    EncoderEXTI_CQ_Init(&g_enc);

    /* L298: ENA=PC8, IN1=PC13, IN2=PE5 */
    L298_Pins_t pins = {
        .ENA_Port = GPIOC, .ENA_Pin = 8,
        .IN1_Port = GPIOC, .IN1_Pin = 13,
        .IN2_Port = GPIOE, .IN2_Pin = 5
    };
    L298_Handle_t mot;
    L298_Init(&mot, &pins, PWM_PERIOD_MS);

    printf("START: L298(PC8/PC13/PE5) + Encoder(PC7/PC9)\n");
    printf("COUNTS_PER_REV=%ld | PWM_PERIOD=%ums\n", (long)COUNTS_PER_REV, (unsigned)PWM_PERIOD_MS);
    printf("Cmd: nhap so vong (vd: 3, -2, 0). RST de reset encoder.\n");
    printf("FAST=%u/255 | SLOW=%u/255 | brake=%ums | slow_band=%ld counts\n",
           (unsigned)SPEED_FAST_8BIT, (unsigned)SPEED_SLOW_8BIT,
           (unsigned)BRAKE_MS, (long)SLOW_BAND_COUNTS);

    /* control state */
    uint8_t running = 0;
    int32_t target_counts = 0;

    uint32_t last_print = millis_ms();
    int32_t last_cnt = EncoderEXTI_CQ_GetCount(&g_enc);
    uint32_t last_move_ms = millis_ms();

    char line[CMD_BUF_SZ];

    while (1)
    {
        uint32_t now = millis_ms();

        /* always run soft PWM task */
        L298_Task(&mot, now);

        /* ========== 1) Receive command ========== */
        if (cmd_poll_line(line, sizeof(line)))
        {
            if (strcmp(line, "RST") == 0 || strcmp(line, "rst") == 0)
            {
                EncoderEXTI_CQ_Reset(&g_enc); /* yêu cầu thư viện encoder có hàm reset */
                printf("OK: encoder reset -> 0\n");
            }
            else
            {
                long rev_cmd = strtol(line, NULL, 10);

                if (rev_cmd == 0)
                {
                    running = 0;
                    L298_Enable(&mot, 0);
                    L298_Coast(&mot);
                    printf("STOP\n");
                }
                else
                {
                    long rev_abs = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
                    target_counts = (int32_t)(rev_abs * (long)COUNTS_PER_REV);

                    EncoderEXTI_CQ_Reset(&g_enc);
                    last_cnt = 0;
                    last_move_ms = now;

                    if (rev_cmd > 0) L298_SetDir(&mot, L298_DIR_FWD);
                    else             L298_SetDir(&mot, L298_DIR_REV);

                    L298_SetSpeed8(&mot, SPEED_FAST_8BIT);
                    L298_Enable(&mot, 1);

                    running = 1;

                    printf("GO: rev=%ld -> target_counts=%ld\n", rev_cmd, (long)target_counts);
                }
            }
        }

        /* ========== 2) Run control if running ========== */
        if (running)
        {
            int32_t cnt = EncoderEXTI_CQ_GetCount(&g_enc);
            int32_t mag = iabs32(cnt);

            /* detect no-move */
            if (cnt != last_cnt) {
                last_cnt = cnt;
                last_move_ms = now;
            } else {
                if ((now - last_move_ms) >= NO_MOVE_RESET_MS) {
                    EncoderEXTI_CQ_Reset(&g_enc);
                    running = 0;
                    L298_Enable(&mot, 0);
                    L298_Coast(&mot);
                    printf("WARN: %ums khong thay xung -> reset 0 va dung\n", (unsigned)NO_MOVE_RESET_MS);
                }
            }

            /* 2-stage speed: FAST -> SLOW near target */
            if (running)
            {
                int32_t remain = target_counts - mag;
                if (remain < 0) remain = 0;

                if (remain <= (int32_t)SLOW_BAND_COUNTS) {
                    L298_SetSpeed8(&mot, SPEED_SLOW_8BIT);
                } else {
                    L298_SetSpeed8(&mot, SPEED_FAST_8BIT);
                }

                /* reached target -> brake then stop */
                if (mag >= target_counts)
                {
                    running = 0;

                    /* brake phase:
                       To brake effectively with L298: IN1=IN2=1 and ENA active.
                       We force duty 100% during brake hold.
                    */
                    L298_SetDutyPercent(&mot, 100);
                    L298_Enable(&mot, 1);
                    L298_Brake(&mot);

                    uint32_t t0 = millis_ms();
                    while ((millis_ms() - t0) < BRAKE_MS) {
                        L298_Task(&mot, millis_ms());
                    }

                    L298_Coast(&mot);
                    L298_Enable(&mot, 0);

                    float rev_done = (float)mag / (float)COUNTS_PER_REV;
                    printf("DONE: cnt=%ld (~%.3f vong)\n", (long)mag, rev_done);
                }
            }
        }
        else
        {
            /* ensure disabled */
            L298_Enable(&mot, 0);
        }

        /* ========== 3) Periodic print ========== */
        if ((now - last_print) >= PRINT_PERIOD_MS)
        {
            last_print = now;

            int32_t c = EncoderEXTI_CQ_GetCount(&g_enc);
            int32_t a = iabs32(c);

            float rev = (float)a / (float)COUNTS_PER_REV;

            int32_t turns = a / (int32_t)COUNTS_PER_REV;
            int32_t rem   = a % (int32_t)COUNTS_PER_REV;

            printf("run=%u | cnt=%ld (abs=%ld) | rev=%.3f | %ld vong + %ld/%ld\n",
                   (unsigned)running,
                   (long)c, (long)a,
                   rev,
                   (long)turns, (long)rem, (long)COUNTS_PER_REV);
        }
    }
}
