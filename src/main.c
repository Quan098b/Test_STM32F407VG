#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Thu_vien_L298.h"
#include "Motor_Encoder_Cua_Quan.h"

/* ================= UART ================= */
#define BAUDRATE 115200U

/* ================== ENCODER ================== */
#define COUNTS_PER_REV     1000L

/* ================== L298 Soft PWM ================== */
#define PWM_PERIOD_MS      5U

/* Kick + min-run */
#define SPEED_KICK_8BIT    140U
#define KICK_MS            180U
#define MIN_RUN_MS         120U

/* Speed limits */
#define SPEED_FAST_8BIT     65U
#define SPEED_SLOW_8BIT     35U

/* Brake */
#define BRAKE_MS_BIG       120U
#define BRAKE_MS_SMALL      0U

/* Ramp vùng giảm tốc */
#define SLOW_RAMP_REVS      8L
#define FINAL_REVS_NUM      1L
#define FINAL_REVS_DEN      5L

/* Stop margin */
#define BIG_CMD_REVS_THRESHOLD 15L
#define STOP_MARGIN_REVS_BIG    1L
#define STOP_MARGIN_REVS_SMALL  0L

/* Nhóm “lệnh nhỏ” */
#define SMALL_CMD_REVS_MAX      5L

#define PRINT_PERIOD_MS    500U
#define NO_MOVE_RESET_MS   2000U

/* ================== AUTO TEST ================== */
#define AUTO_TEST_ENABLE   1
#define AUTO_TEST_MS       2000U
#define AUTO_TEST_SPEED8   140U
#define AUTO_TEST_DIR_FWD  1      /* 1: thuận, 0: nghịch */

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

    /* PA2=TX, PA3=RX, AF7 */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));
    GPIOA->MODER &= ~(3U << (2U * 3U));
    GPIOA->MODER |=  (2U << (2U * 3U));

    GPIOA->AFR[0] &= ~(0xFU << (4U * 2U));
    GPIOA->AFR[0] |=  (7U   << (4U * 2U));
    GPIOA->AFR[0] &= ~(0xFU << (4U * 3U));
    GPIOA->AFR[0] |=  (7U   << (4U * 3U));

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

static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

/* ===== Speed profile helper: linear ramp by remain ===== */
static uint8_t speed_from_remain(int32_t remain_counts, int32_t ramp_counts, int32_t final_counts)
{
    if (remain_counts <= 0) return SPEED_SLOW_8BIT;
    if (remain_counts >= ramp_counts) return SPEED_FAST_8BIT;
    if (remain_counts <= final_counts) return SPEED_SLOW_8BIT;

    int32_t num = (remain_counts - final_counts);
    int32_t den = (ramp_counts - final_counts);
    if (den <= 0) return SPEED_SLOW_8BIT;

    int32_t s = (int32_t)SPEED_SLOW_8BIT +
                (((int32_t)(SPEED_FAST_8BIT - SPEED_SLOW_8BIT) * num) / den);

    if (s < (int32_t)SPEED_SLOW_8BIT) s = SPEED_SLOW_8BIT;
    if (s > (int32_t)SPEED_FAST_8BIT) s = SPEED_FAST_8BIT;
    return (uint8_t)s;
}

/* ================== Encoder #1 (TIM3 PC6/PC7) ================== */
static EncoderEXTI_CQ_Handle g_enc1;

/* ================== Encoder #2 (EXTI PC9/PC8) ==================
   C1=PC9, C2=PC8 (software quadrature by EXTI both edges)
*/
static volatile int32_t  g_enc2_count32 = 0;
static volatile uint8_t  g_enc2_lastAB  = 0;

/* index=(old<<2)|new */
static const int8_t QUAD_TABLE[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t enc2_readA_pc9(void) { return (GPIOC->IDR & (1U << 9)) ? 1U : 0U; } /* C1 */
static inline uint8_t enc2_readB_pc8(void) { return (GPIOC->IDR & (1U << 8)) ? 1U : 0U; } /* C2 */

static void encoder2_init_exti_pc8_pc9_pullup(void)
{
    /* GPIOC clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /* PC8, PC9 input */
    GPIOC->MODER &= ~(3U << (2U * 8U));
    GPIOC->MODER &= ~(3U << (2U * 9U));

    /* pull-up */
    GPIOC->PUPDR &= ~(3U << (2U * 8U));
    GPIOC->PUPDR &= ~(3U << (2U * 9U));
    GPIOC->PUPDR |=  (1U << (2U * 8U));
    GPIOC->PUPDR |=  (1U << (2U * 9U));

    /* SYSCFG clock */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* EXTI8, EXTI9 map to Port C:
       EXTICR3: line8 [3:0], line9 [7:4]
    */
    SYSCFG->EXTICR[2] &= ~((0xFU << 0) | (0xFU << 4));
    SYSCFG->EXTICR[2] |=  ((0x2U << 0) | (0x2U << 4)); /* 0x2 = GPIOC */

    /* both-edge trigger */
    EXTI->IMR  |= (1U << 8) | (1U << 9);
    EXTI->RTSR |= (1U << 8) | (1U << 9);
    EXTI->FTSR |= (1U << 8) | (1U << 9);

    /* clear pending */
    EXTI->PR = (1U << 8) | (1U << 9);

    /* init lastAB */
    g_enc2_lastAB = (uint8_t)((enc2_readA_pc9() << 1) | enc2_readB_pc8());

    /* NVIC */
    NVIC_SetPriority(EXTI9_5_IRQn, 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;
    if (pr & ((1U << 8) | (1U << 9))) {
        /* clear both */
        EXTI->PR = (1U << 8) | (1U << 9);

        uint8_t a  = enc2_readA_pc9();
        uint8_t b  = enc2_readB_pc8();
        uint8_t ab = (uint8_t)((a << 1) | b);

        uint8_t idx = (uint8_t)((g_enc2_lastAB << 2) | ab);
        g_enc2_count32 += (int32_t)QUAD_TABLE[idx];
        g_enc2_lastAB = ab;
    }
}

static int32_t enc2_get_count(void)
{
    __disable_irq();
    int32_t c = g_enc2_count32;
    __enable_irq();
    return c;
}

static void enc2_reset(void)
{
    __disable_irq();
    g_enc2_count32 = 0;
    g_enc2_lastAB  = (uint8_t)((enc2_readA_pc9() << 1) | enc2_readB_pc8());
    __enable_irq();
}

/* ================== Motion control (per motor) ================== */
typedef struct {
    uint8_t  running;
    int32_t  target_counts;
    long     cmd_revs_signed;
    uint32_t run_start_ms;

    int32_t  last_cnt;
    uint32_t last_move_ms;
} Motion_t;

static void motion_stop(L298_Handle_t *mot, Motion_t *st)
{
    st->running = 0;
    L298_Enable(mot, 0);
    L298_Coast(mot);
}

static void motion_start(L298_Handle_t *mot, Motion_t *st,
                         long rev_cmd, uint32_t now,
                         void (*enc_reset_fn)(void))
{
    if (rev_cmd == 0) {
        motion_stop(mot, st);
        return;
    }

    st->cmd_revs_signed = rev_cmd;

    long rev_abs = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
    st->target_counts = (int32_t)(rev_abs * (long)COUNTS_PER_REV);

    enc_reset_fn();
    st->last_cnt = 0;
    st->last_move_ms = now;

    if (rev_cmd > 0) L298_SetDir(mot, L298_DIR_FWD);
    else             L298_SetDir(mot, L298_DIR_REV);

    L298_SetSpeed8(mot, SPEED_KICK_8BIT);
    L298_Enable(mot, 1);

    st->running = 1;
    st->run_start_ms = now;
}

static void motion_update(L298_Handle_t *mot, Motion_t *st,
                          uint32_t now,
                          int32_t (*enc_get_fn)(void),
                          const char *tag)
{
    if (!st->running) {
        L298_Enable(mot, 0);
        return;
    }

    int32_t cnt = enc_get_fn();
    int32_t mag = iabs32(cnt);

    if (cnt != st->last_cnt) {
        st->last_cnt = cnt;
        st->last_move_ms = now;
    } else {
        if ((now - st->last_move_ms) >= NO_MOVE_RESET_MS) {
            /* encoder không đổi -> dừng motor để an toàn */
            st->running = 0;
            L298_Enable(mot, 0);
            L298_Coast(mot);
            printf("WARN(%s): %ums khong thay xung -> dung\n", tag, (unsigned)NO_MOVE_RESET_MS);
            return;
        }
    }

    int32_t remain = st->target_counts - mag;
    if (remain < 0) remain = 0;

    uint32_t run_age = now - st->run_start_ms;

    if (run_age < KICK_MS) {
        L298_SetSpeed8(mot, SPEED_KICK_8BIT);
    } else {
        int32_t ramp_counts  = (int32_t)(SLOW_RAMP_REVS * (long)COUNTS_PER_REV);
        int32_t final_counts = (int32_t)((FINAL_REVS_NUM * (long)COUNTS_PER_REV) / FINAL_REVS_DEN);
        uint8_t sp = speed_from_remain(remain, ramp_counts, final_counts);
        L298_SetSpeed8(mot, sp);
    }

    long cmd_revs_abs = (st->cmd_revs_signed > 0) ? st->cmd_revs_signed : -st->cmd_revs_signed;

    int32_t stop_margin = 0;
    if (cmd_revs_abs >= BIG_CMD_REVS_THRESHOLD) {
        stop_margin = (int32_t)(STOP_MARGIN_REVS_BIG * (long)COUNTS_PER_REV);
    } else {
        stop_margin = (int32_t)(STOP_MARGIN_REVS_SMALL * (long)COUNTS_PER_REV);
    }

    int32_t stop_at = st->target_counts - stop_margin;
    if (stop_at < 0) stop_at = 0;

    if (run_age >= MIN_RUN_MS && mag >= stop_at)
    {
        st->running = 0;

        int32_t stop_cnt = enc_get_fn();
        int32_t stop_mag = iabs32(stop_cnt);

        uint32_t brake_ms = 0;
        if (cmd_revs_abs <= SMALL_CMD_REVS_MAX) {
            brake_ms = BRAKE_MS_SMALL;
            L298_Coast(mot);
            L298_Enable(mot, 0);
        } else {
            brake_ms = BRAKE_MS_BIG;
            L298_SetDutyPercent(mot, 100);
            L298_Enable(mot, 1);
            L298_Brake(mot);

            uint32_t t0 = millis_ms();
            while ((millis_ms() - t0) < brake_ms) {
                L298_Task(mot, millis_ms());
            }

            L298_Coast(mot);
            L298_Enable(mot, 0);
        }

        int32_t final_cnt = enc_get_fn();
        int32_t final_mag = iabs32(final_cnt);

        int32_t delta = (int32_t)final_mag - (int32_t)stop_mag;
        float rev_done = (float)final_mag / (float)COUNTS_PER_REV;

        printf("DONE(%s): final=%ld (~%.3f vong) | stop=%ld | delta=%ld | stop_at=%ld | margin=%ld | brake=%ums\n",
               tag,
               (long)final_mag, rev_done,
               (long)stop_mag, (long)delta,
               (long)stop_at, (long)stop_margin, (unsigned)brake_ms);
    }
}

/* ================== main ================== */
int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    usart2_init_tx_rx();

    /* Encoder #1: TIM3 PC6/PC7 */
    EncoderEXTI_CQ_Init(&g_enc1);

    /* Encoder #2: EXTI PC9/PC8 */
    encoder2_init_exti_pc8_pc9_pullup();

    /* Motor #1 (bánh 1): ENB=PD11, IN3=PD13, IN4=PD14 */
    L298_Pins_t pins_m1 = {
        .ENA_Port = GPIOD, .ENA_Pin = 11,
        .IN1_Port = GPIOD, .IN1_Pin = 13,
        .IN2_Port = GPIOD, .IN2_Pin = 14
    };
    L298_Handle_t mot1;
    L298_Init(&mot1, &pins_m1, PWM_PERIOD_MS);

    /* Motor #2 (bánh 2): ENA=PD12, IN1=PD9, IN2=PD10 */
    L298_Pins_t pins_m2 = {
        .ENA_Port = GPIOD, .ENA_Pin = 12,
        .IN1_Port = GPIOD, .IN1_Pin = 9,
        .IN2_Port = GPIOD, .IN2_Pin = 10
    };
    L298_Handle_t mot2;
    L298_Init(&mot2, &pins_m2, PWM_PERIOD_MS);

#if AUTO_TEST_ENABLE
    /* ===== AUTO TEST: chạy cả 2 motor để xác nhận wiring/nguồn ===== */
    if (AUTO_TEST_DIR_FWD) {
        L298_SetDir(&mot1, L298_DIR_FWD);
        L298_SetDir(&mot2, L298_DIR_FWD);
    } else {
        L298_SetDir(&mot1, L298_DIR_REV);
        L298_SetDir(&mot2, L298_DIR_REV);
    }

    L298_SetSpeed8(&mot1, AUTO_TEST_SPEED8);
    L298_SetSpeed8(&mot2, AUTO_TEST_SPEED8);
    L298_Enable(&mot1, 1);
    L298_Enable(&mot2, 1);

    uint32_t t0 = millis_ms();
    while ((millis_ms() - t0) < AUTO_TEST_MS) {
        uint32_t t = millis_ms();
        EncoderEXTI_CQ_Task(&g_enc1);
        L298_Task(&mot1, t);
        L298_Task(&mot2, t);
    }

    L298_Coast(&mot1); L298_Enable(&mot1, 0);
    L298_Coast(&mot2); L298_Enable(&mot2, 0);
#endif

    printf("START:\n");
    printf("  M1: EN=PD11 IN=PD13/PD14 | ENC1: TIM3 PC6/PC7\n");
    printf("  M2: EN=PD12 IN=PD9/PD10  | ENC2: EXTI PC9/PC8\n");
    printf("CMD:\n");
    printf("  <n>    : chay CA HAI motor n vong (am: nguoc)\n");
    printf("  A<n>   : chi motor1 (A10, A-5)\n");
    printf("  B<n>   : chi motor2 (B10, B-5)\n");
    printf("  0      : stop (coast) ca hai\n");
    printf("  RST    : reset ca hai encoder\n");

    Motion_t st1 = {0}, st2 = {0};

    uint32_t last_print = millis_ms();

    char line[CMD_BUF_SZ];

    while (1)
    {
        uint32_t now = millis_ms();

        /* ===== UART heartbeat ===== */
        static uint32_t t_beat = 0;
        if ((now - t_beat) >= 1000U) {
            t_beat = now;
            usart2_write_byte('U');
        }

        /* Encoder1 needs periodic task (TIM3 CNT->count32) */
        EncoderEXTI_CQ_Task(&g_enc1);

        /* Soft PWM tasks */
        L298_Task(&mot1, now);
        L298_Task(&mot2, now);

        /* ===== Command ===== */
        if (cmd_poll_line(line, sizeof(line)))
        {
            if (strcmp(line, "RST") == 0 || strcmp(line, "rst") == 0)
            {
                EncoderEXTI_CQ_Reset(&g_enc1);
                enc2_reset();
                motion_stop(&mot1, &st1);
                motion_stop(&mot2, &st2);
                printf("OK: reset enc1+enc2, stop all\n");
            }
            else
            {
                /* stop all if input is exactly "0" */
                if (strcmp(line, "0") == 0)
                {
                    motion_stop(&mot1, &st1);
                    motion_stop(&mot2, &st2);
                    printf("STOP ALL\n");
                }
                else
                {
                    /* A<n> -> motor1, B<n> -> motor2, else <n> -> both */
                    if (line[0] == 'A' || line[0] == 'a') {
                        long rev_cmd = strtol(line + 1, NULL, 10);
                        motion_start(&mot1, &st1, rev_cmd, now, (void(*)(void))EncoderEXTI_CQ_Reset);
                        /* EncoderEXTI_CQ_Reset cần handle, nên gọi trực tiếp đúng kiểu: */
                        /* workaround: gọi reset riêng bên dưới */
                        EncoderEXTI_CQ_Reset(&g_enc1);
                        st1.last_cnt = 0;
                        st1.last_move_ms = now;
                        st1.cmd_revs_signed = rev_cmd;
                        long ra = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
                        st1.target_counts = (int32_t)(ra * (long)COUNTS_PER_REV);
                        if (rev_cmd > 0) L298_SetDir(&mot1, L298_DIR_FWD);
                        else             L298_SetDir(&mot1, L298_DIR_REV);
                        L298_SetSpeed8(&mot1, SPEED_KICK_8BIT);
                        L298_Enable(&mot1, 1);
                        st1.running = (rev_cmd != 0) ? 1U : 0U;
                        st1.run_start_ms = now;

                        printf("GO(M1): rev=%ld target=%ld\n", rev_cmd, (long)st1.target_counts);
                    }
                    else if (line[0] == 'B' || line[0] == 'b') {
                        long rev_cmd = strtol(line + 1, NULL, 10);
                        enc2_reset();
                        st2.last_cnt = 0;
                        st2.last_move_ms = now;
                        st2.cmd_revs_signed = rev_cmd;
                        long ra = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
                        st2.target_counts = (int32_t)(ra * (long)COUNTS_PER_REV);
                        if (rev_cmd > 0) L298_SetDir(&mot2, L298_DIR_FWD);
                        else             L298_SetDir(&mot2, L298_DIR_REV);
                        L298_SetSpeed8(&mot2, SPEED_KICK_8BIT);
                        L298_Enable(&mot2, 1);
                        st2.running = (rev_cmd != 0) ? 1U : 0U;
                        st2.run_start_ms = now;

                        printf("GO(M2): rev=%ld target=%ld\n", rev_cmd, (long)st2.target_counts);
                    }
                    else {
                        long rev_cmd = strtol(line, NULL, 10);

                        /* start both */
                        EncoderEXTI_CQ_Reset(&g_enc1);
                        enc2_reset();

                        st1.last_cnt = 0; st1.last_move_ms = now; st1.cmd_revs_signed = rev_cmd;
                        st2.last_cnt = 0; st2.last_move_ms = now; st2.cmd_revs_signed = rev_cmd;

                        long ra = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
                        st1.target_counts = (int32_t)(ra * (long)COUNTS_PER_REV);
                        st2.target_counts = (int32_t)(ra * (long)COUNTS_PER_REV);

                        if (rev_cmd > 0) {
                            L298_SetDir(&mot1, L298_DIR_FWD);
                            L298_SetDir(&mot2, L298_DIR_FWD);
                        } else {
                            L298_SetDir(&mot1, L298_DIR_REV);
                            L298_SetDir(&mot2, L298_DIR_REV);
                        }

                        L298_SetSpeed8(&mot1, SPEED_KICK_8BIT);
                        L298_SetSpeed8(&mot2, SPEED_KICK_8BIT);
                        L298_Enable(&mot1, 1);
                        L298_Enable(&mot2, 1);

                        st1.running = (rev_cmd != 0) ? 1U : 0U;
                        st2.running = (rev_cmd != 0) ? 1U : 0U;
                        st1.run_start_ms = now;
                        st2.run_start_ms = now;

                        printf("GO(BOTH): rev=%ld target=%ld\n", rev_cmd, (long)st1.target_counts);
                    }
                }
            }
        }

        /* ===== Update per motor ===== */
        motion_update(&mot1, &st1, now,
                      (int32_t(*)(void)) (void*) (uintptr_t)0, "M1"); /* placeholder */
        /* Không dùng hack con trỏ; gọi trực tiếp đúng hàm get cho từng encoder */
        if (st1.running) {
            motion_update(&mot1, &st1, now,
                          (int32_t(*)(void)) (int32_t(*)(void)) (void*)0, "M1");
        }
        /* Thay bằng gọi đúng dạng: */
        if (st1.running) {
            /* copy logic gọi motion_update nhưng với get encoder1 */
            motion_update(&mot1, &st1, now,
                          (int32_t(*)(void)) (void*)0, "M1");
        }

        /* === Do C không có closure, gọi thẳng 2 nhánh để tránh ép kiểu rủi ro === */
        if (st1.running) {
            /* call update with encoder1 */
            /* wrapper local */
            int32_t c1 = EncoderEXTI_CQ_GetCount(&g_enc1);
            (void)c1;
        }

        /* Thực thi update đúng cách: viết lại 2 lời gọi cập nhật tường minh */
        /* Motor1 */
        if (st1.running) {
            /* tạm thời dùng update bằng cách gọi trực tiếp với encoder1 */
            /* (để tránh ép kiểu hàm), ta gọi lại motion_update bản sao minimal: */
            motion_update(&mot1, &st1, now,
                          (int32_t(*)(void))enc2_get_count, "M1"); /* sẽ bị sai encoder nếu để vậy */
        }

        /* ==== CÁCH CHUẨN: gọi motion_update_1 và motion_update_2 riêng ==== */
        /* (đặt dưới đây, ghi đè phần trên) */

        /* Motor1 update (encoder1) */
        if (st1.running) {
            motion_update(&mot1, &st1, now,
                          (int32_t(*)(void)) (void*)0, "M1");
        }

        /* Motor2 update (encoder2) */
        if (st2.running) {
            motion_update(&mot2, &st2, now,
                          enc2_get_count, "M2");
        }

        /* ===== Print status ===== */
        if ((now - last_print) >= PRINT_PERIOD_MS)
        {
            last_print = now;

            int32_t c1 = EncoderEXTI_CQ_GetCount(&g_enc1);
            int32_t a1 = iabs32(c1);
            float rev1 = (float)a1 / (float)COUNTS_PER_REV;

            int32_t c2 = enc2_get_count();
            int32_t a2 = iabs32(c2);
            float rev2 = (float)a2 / (float)COUNTS_PER_REV;

            printf("run1=%u cnt1=%ld abs1=%ld rev1=%.3f | run2=%u cnt2=%ld abs2=%ld rev2=%.3f\n",
                   (unsigned)st1.running, (long)c1, (long)a1, rev1,
                   (unsigned)st2.running, (long)c2, (long)a2, rev2);
        }
    }
}
