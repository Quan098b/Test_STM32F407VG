#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BAUDRATE 115200U

/* ================== CHỈNH Ở ĐÂY ================== */
/* Encoder: nếu 600 PPR (1 kênh) và decode x4 => 2400 count/vòng */
#define ENCODER_PPR        600L
#define ENCODER_QUAD_MULT  4L
#define COUNTS_PER_REV     (ENCODER_PPR * ENCODER_QUAD_MULT)

/* PWM mềm cho ENA (PC8): giảm tốc */
#define PWM_PERIOD_MS      5U      /* 5ms ~ 200Hz */
#define SPEED_DUTY_PERCENT 50U     /* 0..100 (%). Giảm để chậm hơn */

/* In trạng thái mỗi bao lâu */
#define PRINT_PERIOD_MS    500U

/* Nếu 2 giây không có xung encoder khi đang chạy -> reset và dừng */
#define NO_MOVE_RESET_MS   2000U
/* ================================================= */

/* ================== SysTick ms tick ================== */
static volatile uint32_t g_ms = 0;
void SysTick_Handler(void) { g_ms++; }
static inline uint32_t millis_ms(void) { return g_ms; }

/* ================== USART2 TX/RX (PA2/PA3) ================== */
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

    /* PA2 = AF7 (USART2_TX), PA3 = AF7 (USART2_RX) */
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

    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;  /* TX+RX */
    USART2->CR1 |= USART_CR1_UE;
}

static void usart2_write_byte(uint8_t b)
{
    while (!(USART2->SR & USART_SR_TXE)) { }
    USART2->DR = b;
}

/* printf() -> USART2 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') usart2_write_byte('\r');
        usart2_write_byte((uint8_t)ptr[i]);
    }
    return len;
}

/* Poll RX: trả về -1 nếu chưa có byte */
static int usart2_read_byte_nonblock(void)
{
    if (USART2->SR & USART_SR_RXNE) {
        return (int)(uint8_t)USART2->DR;
    }
    return -1;
}

/* ================== L298 GPIO: ENA=PC8, IN1=PC13, IN2=PE5 ================== */
static void l298_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    /* PC8 output (ENA) */
    GPIOC->MODER &= ~(3U << (8U * 2U));
    GPIOC->MODER |=  (1U << (8U * 2U));
    GPIOC->OTYPER &= ~(1U << 8U);
    GPIOC->PUPDR  &= ~(3U << (8U * 2U));

    /* PC13 output (IN1) */
    GPIOC->MODER &= ~(3U << (13U * 2U));
    GPIOC->MODER |=  (1U << (13U * 2U));
    GPIOC->OTYPER &= ~(1U << 13U);
    GPIOC->PUPDR  &= ~(3U << (13U * 2U));

    /* PE5 output (IN2) */
    GPIOE->MODER &= ~(3U << (5U * 2U));
    GPIOE->MODER |=  (1U << (5U * 2U));
    GPIOE->OTYPER &= ~(1U << 5U);
    GPIOE->PUPDR  &= ~(3U << (5U * 2U));
}

static void l298_set_dir_forward(void)
{
    /* IN1=1, IN2=0 */
    GPIOC->BSRR = (1U << 13U);
    GPIOE->BSRR = (1U << (5U + 16U));
}

static void l298_set_dir_reverse(void)
{
    /* IN1=0, IN2=1 */
    GPIOC->BSRR = (1U << (13U + 16U));
    GPIOE->BSRR = (1U << 5U);
}

static void l298_coast(void)
{
    /* IN1=0, IN2=0 */
    GPIOC->BSRR = (1U << (13U + 16U));
    GPIOE->BSRR = (1U << (5U + 16U));
}

static void l298_enable(uint8_t en)
{
    if (en) GPIOC->BSRR = (1U << 8U);
    else    GPIOC->BSRR = (1U << (8U + 16U));
}

/* ================== Encoder EXTI: C1=PC7, C2=PC9 ================== */
static volatile int32_t g_enc_count = 0;
static volatile uint8_t g_enc_prev = 0;

/* LUT quadrature (state=(B<<1)|A) */
static const int8_t ENC_LUT[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t enc_read_ab(void)
{
    /* A=PC7, B=PC9 */
    uint8_t a = (GPIOC->IDR & (1U << 7U)) ? 1U : 0U;
    uint8_t b = (GPIOC->IDR & (1U << 9U)) ? 1U : 0U;
    return (uint8_t)((b << 1) | a);
}

static void encoder_exti_init_pc7_pc9(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PC7, PC9 input + pull-up */
    GPIOC->MODER &= ~(3U << (7U * 2U));
    GPIOC->MODER &= ~(3U << (9U * 2U));
    GPIOC->PUPDR &= ~(3U << (7U * 2U));
    GPIOC->PUPDR &= ~(3U << (9U * 2U));
    GPIOC->PUPDR |=  (1U << (7U * 2U));
    GPIOC->PUPDR |=  (1U << (9U * 2U));

    /* Map EXTI7 -> PC, EXTI9 -> PC */
    SYSCFG->EXTICR[1] &= ~(0xFU << 12);
    SYSCFG->EXTICR[1] |=  (0x2U << 12);

    SYSCFG->EXTICR[2] &= ~(0xFU << 4);
    SYSCFG->EXTICR[2] |=  (0x2U << 4);

    /* EXTI mask + both edges */
    EXTI->IMR  |= (1U << 7U) | (1U << 9U);
    EXTI->RTSR |= (1U << 7U) | (1U << 9U);
    EXTI->FTSR |= (1U << 7U) | (1U << 9U);

    EXTI->PR = (1U << 7U) | (1U << 9U);

    g_enc_prev = enc_read_ab();

    NVIC_SetPriority(EXTI9_5_IRQn, 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;

    if (pr & ((1U << 7U) | (1U << 9U)))
    {
        EXTI->PR = (1U << 7U) | (1U << 9U);

        uint8_t curr = enc_read_ab();
        uint8_t idx  = (uint8_t)((g_enc_prev << 2) | curr);
        int8_t  d    = ENC_LUT[idx];

        if (d) g_enc_count += d;
        g_enc_prev = curr;
    }
}

static inline int32_t encoder_get_count(void)
{
    __disable_irq();
    int32_t c = g_enc_count;
    __enable_irq();
    return c;
}

static inline void encoder_reset_count(void)
{
    __disable_irq();
    g_enc_count = 0;
    __enable_irq();
}

/* ================== Command parser: gửi số vòng ================== */
#define CMD_BUF_SZ 32
static char g_cmd_buf[CMD_BUF_SZ];
static uint8_t g_cmd_len = 0;

/* Trả về 1 nếu vừa nhận xong 1 dòng lệnh (kết thúc bởi \n hoặc \r) */
static int cmd_poll_line(char *out, uint32_t out_sz)
{
    int ch;
    while ((ch = usart2_read_byte_nonblock()) >= 0)
    {
        if (ch == '\r' || ch == '\n')
        {
            if (g_cmd_len == 0) return 0; /* bỏ dòng rỗng */

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

/* ================== Main: quay đúng N vòng ================== */
int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    usart2_init_tx_rx();
    l298_gpio_init();
    encoder_exti_init_pc7_pc9();

    printf("START: L298(PC8/PC13/PE5) + Encoder(PC7/PC9)\n");
    printf("COUNTS_PER_REV=%ld | PWM=%ums | duty=%u%%\n",
           (long)COUNTS_PER_REV,
           (unsigned)PWM_PERIOD_MS,
           (unsigned)SPEED_DUTY_PERCENT);
    printf("Nhap so vong (vd: 3, -2, 0). Enter de gui.\n");

    /* trạng thái điều khiển */
    uint8_t move_running = 0;
    int32_t target_counts = 0;

    uint32_t last_print_ms = millis_ms();

    int32_t last_cnt = encoder_get_count();
    uint32_t last_move_ms = millis_ms();

    uint8_t last_ena = 2;

    char line[CMD_BUF_SZ];

    while (1)
    {
        uint32_t now = millis_ms();

        /* ===== 1) Nhận lệnh ===== */
        if (cmd_poll_line(line, sizeof(line)))
        {
            /* Cho phép lệnh RST */
            if (strcmp(line, "RST") == 0 || strcmp(line, "rst") == 0)
            {
                encoder_reset_count();
                printf("OK: reset encoder -> 0\n");
            }
            else
            {
                long rev_cmd = strtol(line, NULL, 10);

                if (rev_cmd == 0)
                {
                    /* dừng */
                    move_running = 0;
                    l298_enable(0);
                    l298_coast();
                    last_ena = 0;
                    printf("STOP\n");
                }
                else
                {
                    /* bắt đầu quay N vòng */
                    long rev_abs = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
                    target_counts = (int32_t)(rev_abs * (long)COUNTS_PER_REV);

                    encoder_reset_count();
                    last_cnt = 0;
                    last_move_ms = now;

                    if (rev_cmd > 0) l298_set_dir_forward();
                    else             l298_set_dir_reverse();

                    move_running = 1;

                    printf("GO: rev=%ld -> target_counts=%ld\n",
                           rev_cmd, (long)target_counts);
                }
            }
        }

        /* ===== 2) Nếu đang chạy: PWM mềm + kiểm tra đủ vòng thì dừng ===== */
        if (move_running)
        {
            /* PWM mềm ENA */
            uint32_t phase = now % PWM_PERIOD_MS;
            uint32_t on_ms = (PWM_PERIOD_MS * (uint32_t)SPEED_DUTY_PERCENT) / 100U;
            uint8_t ena = (phase < on_ms) ? 1U : 0U;

            if (ena != last_ena) {
                l298_enable(ena);
                last_ena = ena;
            }

            /* tiến độ: dùng abs để không bị “âm vòng” */
            int32_t cnt = encoder_get_count();
            int32_t mag = (cnt < 0) ? -cnt : cnt;

            /* phát hiện kẹt/không xung */
            if (cnt != last_cnt) {
                last_cnt = cnt;
                last_move_ms = now;
            } else {
                if ((now - last_move_ms) >= NO_MOVE_RESET_MS) {
                    encoder_reset_count();
                    move_running = 0;
                    l298_enable(0);
                    l298_coast();
                    last_ena = 0;
                    printf("WARN: %ums khong thay xung -> reset 0 va dung\n",
                           (unsigned)NO_MOVE_RESET_MS);
                }
            }

            /* đủ vòng -> dừng */
            if (move_running && (mag >= target_counts))
            {
                move_running = 0;
                l298_enable(0);
                l298_coast();
                last_ena = 0;

                float rev_done = (float)mag / (float)COUNTS_PER_REV;
                printf("DONE: cnt=%ld (~%.3f vong)\n", (long)mag, rev_done);
            }
        }
        else
        {
            /* không chạy thì giữ ENA=0 */
            if (last_ena != 0) {
                l298_enable(0);
                last_ena = 0;
            }
        }

        /* ===== 3) In trạng thái định kỳ ===== */
        if ((now - last_print_ms) >= PRINT_PERIOD_MS)
        {
            last_print_ms = now;

            int32_t c = encoder_get_count();
            int32_t c_abs = (c < 0) ? -c : c;

            float rev = (float)c_abs / (float)COUNTS_PER_REV;

            int32_t turns = c_abs / (int32_t)COUNTS_PER_REV;
            int32_t rem   = c_abs % (int32_t)COUNTS_PER_REV;

            printf("run=%u | cnt=%ld (abs=%ld) | rev=%.3f | %ld vong + %ld/%ld\n",
                   (unsigned)move_running,
                   (long)c, (long)c_abs,
                   rev,
                   (long)turns, (long)rem, (long)COUNTS_PER_REV);
        }
    }
}
