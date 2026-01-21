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

#define SMALL_CMD_REVS_MAX      5L

#define PRINT_PERIOD_MS    500U
#define NO_MOVE_RESET_MS   2000U

/* ================== AUTO TEST ================== */
#define AUTO_TEST_ENABLE   1
#define AUTO_TEST_MS       2000U
#define AUTO_TEST_SPEED8   140U
#define AUTO_TEST_DIR_FWD  1      /* 1: thuận, 0: nghịch */

/* ================== SYNC (P-coupling) ==================
   Nếu bạn muốn CHỈ dùng bias/gain, set SYNC_ENABLE=0
*/
#define SYNC_ENABLE        1
#define SYNC_ERR_DIV       200
#define SYNC_MAX_CORR_8BIT 40

#define SYNC_MIN_8BIT      20
#define SYNC_MAX_8BIT      200

/* ================== CÁCH B: BIAS/GAIN ==================
   - GAIN: % (100 = giữ nguyên, 105 = +5%, 95 = -5%)
   - BIAS: cộng/trừ trực tiếp vào duty8 (0..255)
   Tuning:
     - Nếu bánh A chậm hơn B: tăng M1_GAIN hoặc tăng M1_BIAS (hoặc giảm của B).
*/
#define M1_GAIN_PCT        100
#define M2_GAIN_PCT        100
#define M1_BIAS_8BIT       0
#define M2_BIAS_8BIT       0

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

static uint8_t clamp_u8(int32_t x)
{
    if (x < 0) return 0;
    if (x > 255) return 255;
    return (uint8_t)x;
}

/* Apply fixed calibration (CÁCH B) */
static uint8_t apply_calib_m1(uint8_t duty)
{
    int32_t d = (int32_t)duty;
    d = (d * (int32_t)M1_GAIN_PCT) / 100;
    d += (int32_t)M1_BIAS_8BIT;
    /* giữ trong khoảng sync min/max trước khi clamp 0..255 */
    if (d < (int32_t)SYNC_MIN_8BIT) d = (int32_t)SYNC_MIN_8BIT;
    if (d > (int32_t)SYNC_MAX_8BIT) d = (int32_t)SYNC_MAX_8BIT;
    return clamp_u8(d);
}

static uint8_t apply_calib_m2(uint8_t duty)
{
    int32_t d = (int32_t)duty;
    d = (d * (int32_t)M2_GAIN_PCT) / 100;
    d += (int32_t)M2_BIAS_8BIT;
    if (d < (int32_t)SYNC_MIN_8BIT) d = (int32_t)SYNC_MIN_8BIT;
    if (d > (int32_t)SYNC_MAX_8BIT) d = (int32_t)SYNC_MAX_8BIT;
    return clamp_u8(d);
}

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
static int32_t enc1_get_count(void) { return EncoderEXTI_CQ_GetCount(&g_enc1); }
static void    enc1_reset(void)     { EncoderEXTI_CQ_Reset(&g_enc1); }

/* ================== Encoder #2 (EXTI PC9/PC8) ================== */
static volatile int32_t  g_enc2_count32 = 0;
static volatile uint8_t  g_enc2_lastAB  = 0;

static const int8_t QUAD_TABLE[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t enc2_readA_pc9(void) { return (GPIOC->IDR & (1U << 9)) ? 1U : 0U; }
static inline uint8_t enc2_readB_pc8(void) { return (GPIOC->IDR & (1U << 8)) ? 1U : 0U; }

static void encoder2_init_exti_pc8_pc9_pullup(void)
{
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

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* EXTI8/9 -> GPIOC */
    SYSCFG->EXTICR[2] &= ~((0xFU << 0) | (0xFU << 4));
    SYSCFG->EXTICR[2] |=  ((0x2U << 0) | (0x2U << 4));

    EXTI->IMR  |= (1U << 8) | (1U << 9);
    EXTI->RTSR |= (1U << 8) | (1U << 9);
    EXTI->FTSR |= (1U << 8) | (1U << 9);
    EXTI->PR = (1U << 8) | (1U << 9);

    g_enc2_lastAB = (uint8_t)((enc2_readA_pc9() << 1) | enc2_readB_pc8());

    NVIC_SetPriority(EXTI9_5_IRQn, 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;
    if (pr & ((1U << 8) | (1U << 9))) {
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

/* ================== Motion state ================== */
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

static void motion_start_single(L298_Handle_t *mot, Motion_t *st,
                                long rev_cmd, uint32_t now,
                                void (*enc_reset_fn)(void),
                                uint8_t is_m1)
{
    if (rev_cmd == 0) {
        motion_stop(mot, st);
        return;
    }

    st->cmd_revs_signed = rev_cmd;

    long ra = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
    st->target_counts = (int32_t)(ra * (long)COUNTS_PER_REV);

    enc_reset_fn();
    st->last_cnt = 0;
    st->last_move_ms = now;

    if (rev_cmd > 0) L298_SetDir(mot, L298_DIR_FWD);
    else             L298_SetDir(mot, L298_DIR_REV);

    {
        uint8_t duty = SPEED_KICK_8BIT;
        duty = is_m1 ? apply_calib_m1(duty) : apply_calib_m2(duty);
        L298_SetSpeed8(mot, duty);
    }
    L298_Enable(mot, 1);

    st->running = 1;
    st->run_start_ms = now;
}

static void motion_update_single(L298_Handle_t *mot, Motion_t *st,
                                 uint32_t now,
                                 int32_t (*enc_get_fn)(void),
                                 const char *tag,
                                 uint8_t is_m1)
{
    if (!st->running) { L298_Enable(mot, 0); return; }

    int32_t cnt = enc_get_fn();
    int32_t mag = iabs32(cnt);

    if (cnt != st->last_cnt) { st->last_cnt = cnt; st->last_move_ms = now; }
    else if ((now - st->last_move_ms) >= NO_MOVE_RESET_MS) {
        st->running = 0;
        L298_Enable(mot, 0);
        L298_Coast(mot);
        printf("WARN(%s): %ums khong thay xung -> dung\n", tag, (unsigned)NO_MOVE_RESET_MS);
        return;
    }

    int32_t remain = st->target_counts - mag;
    if (remain < 0) remain = 0;

    uint32_t run_age = now - st->run_start_ms;

    uint8_t base;
    if (run_age < KICK_MS) base = SPEED_KICK_8BIT;
    else {
        int32_t ramp_counts  = (int32_t)(SLOW_RAMP_REVS * (long)COUNTS_PER_REV);
        int32_t final_counts = (int32_t)((FINAL_REVS_NUM * (long)COUNTS_PER_REV) / FINAL_REVS_DEN);
        base = speed_from_remain(remain, ramp_counts, final_counts);
    }

    base = is_m1 ? apply_calib_m1(base) : apply_calib_m2(base);
    L298_SetSpeed8(mot, base);

    long cmd_abs = (st->cmd_revs_signed > 0) ? st->cmd_revs_signed : -st->cmd_revs_signed;

    int32_t stop_margin = 0;
    if (cmd_abs >= BIG_CMD_REVS_THRESHOLD) stop_margin = (int32_t)(STOP_MARGIN_REVS_BIG   * (long)COUNTS_PER_REV);
    else                                  stop_margin = (int32_t)(STOP_MARGIN_REVS_SMALL * (long)COUNTS_PER_REV);

    int32_t stop_at = st->target_counts - stop_margin;
    if (stop_at < 0) stop_at = 0;

    if (run_age >= MIN_RUN_MS && mag >= stop_at)
    {
        st->running = 0;

        uint32_t brake_ms = 0;
        if (cmd_abs <= SMALL_CMD_REVS_MAX) {
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

        printf("DONE(%s): brake=%ums\n", tag, (unsigned)brake_ms);
    }
}

/* ================== SYNC BOTH (P + calibration) ================== */
static uint8_t g_sync_both = 0;

static void start_both(L298_Handle_t *m1, L298_Handle_t *m2,
                       Motion_t *s1, Motion_t *s2,
                       long rev_cmd, uint32_t now)
{
    if (rev_cmd == 0) {
        motion_stop(m1, s1);
        motion_stop(m2, s2);
        g_sync_both = 0;
        return;
    }

    enc1_reset();
    enc2_reset();

    long ra = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
    int32_t target = (int32_t)(ra * (long)COUNTS_PER_REV);

    s1->cmd_revs_signed = rev_cmd;
    s2->cmd_revs_signed = rev_cmd;
    s1->target_counts = target;
    s2->target_counts = target;

    s1->last_cnt = 0; s2->last_cnt = 0;
    s1->last_move_ms = now; s2->last_move_ms = now;

    if (rev_cmd > 0) {
        L298_SetDir(m1, L298_DIR_FWD);
        L298_SetDir(m2, L298_DIR_FWD);
    } else {
        L298_SetDir(m1, L298_DIR_REV);
        L298_SetDir(m2, L298_DIR_REV);
    }

    {
        uint8_t d1 = apply_calib_m1(SPEED_KICK_8BIT);
        uint8_t d2 = apply_calib_m2(SPEED_KICK_8BIT);
        L298_SetSpeed8(m1, d1);
        L298_SetSpeed8(m2, d2);
    }
    L298_Enable(m1, 1);
    L298_Enable(m2, 1);

    s1->running = 1;
    s2->running = 1;
    s1->run_start_ms = now;
    s2->run_start_ms = now;

    g_sync_both = 1;
}

static void sync_update(L298_Handle_t *m1, L298_Handle_t *m2,
                        Motion_t *s1, Motion_t *s2,
                        uint32_t now)
{
#if !SYNC_ENABLE
    motion_update_single(m1, s1, now, enc1_get_count, "A", 1);
    motion_update_single(m2, s2, now, enc2_get_count, "B", 0);
    return;
#endif

    if (!s1->running || !s2->running) { g_sync_both = 0; return; }

    int32_t c1 = enc1_get_count();
    int32_t c2 = enc2_get_count();
    int32_t mag1 = iabs32(c1);
    int32_t mag2 = iabs32(c2);

    if (c1 != s1->last_cnt) { s1->last_cnt = c1; s1->last_move_ms = now; }
    if (c2 != s2->last_cnt) { s2->last_cnt = c2; s2->last_move_ms = now; }

    if ((now - s1->last_move_ms) >= NO_MOVE_RESET_MS ||
        (now - s2->last_move_ms) >= NO_MOVE_RESET_MS) {
        motion_stop(m1, s1);
        motion_stop(m2, s2);
        g_sync_both = 0;
        printf("WARN(SYNC): %ums khong thay xung -> stop both\n", (unsigned)NO_MOVE_RESET_MS);
        return;
    }

    uint32_t run_age = now - s1->run_start_ms;

    int32_t prog = (mag1 + mag2) / 2;
    int32_t remain = s1->target_counts - prog;
    if (remain < 0) remain = 0;

    uint8_t base;
    if (run_age < KICK_MS) base = SPEED_KICK_8BIT;
    else {
        int32_t ramp_counts  = (int32_t)(SLOW_RAMP_REVS * (long)COUNTS_PER_REV);
        int32_t final_counts = (int32_t)((FINAL_REVS_NUM * (long)COUNTS_PER_REV) / FINAL_REVS_DEN);
        base = speed_from_remain(remain, ramp_counts, final_counts);
    }

    /* P-coupling theo sai lệch tiến độ */
    int32_t err  = mag1 - mag2;         /* >0: A đi trước */
    int32_t corr = err / SYNC_ERR_DIV;

    if (corr >  (int32_t)SYNC_MAX_CORR_8BIT) corr =  (int32_t)SYNC_MAX_CORR_8BIT;
    if (corr < -(int32_t)SYNC_MAX_CORR_8BIT) corr = -(int32_t)SYNC_MAX_CORR_8BIT;

    int32_t d1 = (int32_t)base - corr;
    int32_t d2 = (int32_t)base + corr;

    /* CÁCH B: áp hiệu chỉnh cố định sau khi đã có d1/d2 */
    uint8_t out1 = apply_calib_m1(clamp_u8(d1));
    uint8_t out2 = apply_calib_m2(clamp_u8(d2));

    L298_SetSpeed8(m1, out1);
    L298_SetSpeed8(m2, out2);

    /* điều kiện dừng chung */
    long cmd_abs = (s1->cmd_revs_signed > 0) ? s1->cmd_revs_signed : -s1->cmd_revs_signed;

    int32_t stop_margin = 0;
    if (cmd_abs >= BIG_CMD_REVS_THRESHOLD) stop_margin = (int32_t)(STOP_MARGIN_REVS_BIG   * (long)COUNTS_PER_REV);
    else                                  stop_margin = (int32_t)(STOP_MARGIN_REVS_SMALL * (long)COUNTS_PER_REV);

    int32_t stop_at = s1->target_counts - stop_margin;
    if (stop_at < 0) stop_at = 0;

    if (run_age >= MIN_RUN_MS && prog >= stop_at)
    {
        s1->running = 0;
        s2->running = 0;
        g_sync_both = 0;

        uint32_t brake_ms = 0;
        if (cmd_abs <= SMALL_CMD_REVS_MAX) {
            brake_ms = BRAKE_MS_SMALL;
            L298_Coast(m1); L298_Enable(m1, 0);
            L298_Coast(m2); L298_Enable(m2, 0);
        } else {
            brake_ms = BRAKE_MS_BIG;

            L298_SetDutyPercent(m1, 100); L298_Enable(m1, 1); L298_Brake(m1);
            L298_SetDutyPercent(m2, 100); L298_Enable(m2, 1); L298_Brake(m2);

            uint32_t t0 = millis_ms();
            while ((millis_ms() - t0) < brake_ms) {
                uint32_t t = millis_ms();
                L298_Task(m1, t);
                L298_Task(m2, t);
            }

            L298_Coast(m1); L298_Enable(m1, 0);
            L298_Coast(m2); L298_Enable(m2, 0);
        }

        printf("DONE(SYNC+B): cA=%ld cB=%ld | d=%ld | brake=%ums\n",
               (long)c1, (long)c2, (long)(iabs32(c1) - iabs32(c2)), (unsigned)brake_ms);
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

    /* Motor A (M1): EN=PD11, IN=PD13/PD14 */
    L298_Pins_t pins_m1 = {
        .ENA_Port = GPIOD, .ENA_Pin = 11,
        .IN1_Port = GPIOD, .IN1_Pin = 13,
        .IN2_Port = GPIOD, .IN2_Pin = 14
    };
    L298_Handle_t mot1;
    L298_Init(&mot1, &pins_m1, PWM_PERIOD_MS);

    /* Motor B (M2): EN=PD12, IN=PD9/PD10 */
    L298_Pins_t pins_m2 = {
        .ENA_Port = GPIOD, .ENA_Pin = 12,
        .IN1_Port = GPIOD, .IN1_Pin = 9,
        .IN2_Port = GPIOD, .IN2_Pin = 10
    };
    L298_Handle_t mot2;
    L298_Init(&mot2, &pins_m2, PWM_PERIOD_MS);

#if AUTO_TEST_ENABLE
    if (AUTO_TEST_DIR_FWD) {
        L298_SetDir(&mot1, L298_DIR_FWD);
        L298_SetDir(&mot2, L298_DIR_FWD);
    } else {
        L298_SetDir(&mot1, L298_DIR_REV);
        L298_SetDir(&mot2, L298_DIR_REV);
    }

    L298_SetSpeed8(&mot1, apply_calib_m1(AUTO_TEST_SPEED8));
    L298_SetSpeed8(&mot2, apply_calib_m2(AUTO_TEST_SPEED8));
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
    printf("  A(M1): EN=PD11 IN=PD13/PD14 | ENC1: TIM3 PC6/PC7\n");
    printf("  B(M2): EN=PD12 IN=PD9/PD10  | ENC2: EXTI PC9/PC8\n");
    printf("CALIB:\n");
    printf("  M1: gain=%d%% bias=%d | M2: gain=%d%% bias=%d\n",
           (int)M1_GAIN_PCT, (int)M1_BIAS_8BIT, (int)M2_GAIN_PCT, (int)M2_BIAS_8BIT);
    printf("SYNC_ENABLE=%d\n", (int)SYNC_ENABLE);
    printf("CMD:\n");
    printf("  <n>    : chay dong bo 2 banh n vong (am: nguoc)\n");
    printf("  A<n>   : chi banh A\n");
    printf("  B<n>   : chi banh B\n");
    printf("  0      : stop\n");
    printf("  RST    : reset 2 encoder\n");

    Motion_t st1 = {0}, st2 = {0};
    uint32_t last_print = millis_ms();
    char line[CMD_BUF_SZ];

    while (1)
    {
        uint32_t now = millis_ms();

        static uint32_t t_beat = 0;
        if ((now - t_beat) >= 1000U) {
            t_beat = now;
            usart2_write_byte('U');
        }

        EncoderEXTI_CQ_Task(&g_enc1);
        L298_Task(&mot1, now);
        L298_Task(&mot2, now);

        if (cmd_poll_line(line, sizeof(line)))
        {
            if (strcmp(line, "RST") == 0 || strcmp(line, "rst") == 0)
            {
                enc1_reset();
                enc2_reset();
                motion_stop(&mot1, &st1);
                motion_stop(&mot2, &st2);
                g_sync_both = 0;
                printf("OK: reset enc1+enc2, stop all\n");
            }
            else if (strcmp(line, "0") == 0)
            {
                motion_stop(&mot1, &st1);
                motion_stop(&mot2, &st2);
                g_sync_both = 0;
                printf("STOP ALL\n");
            }
            else if (line[0] == 'A' || line[0] == 'a')
            {
                long rev_cmd = strtol(line + 1, NULL, 10);
                g_sync_both = 0;
                motion_start_single(&mot1, &st1, rev_cmd, now, enc1_reset, 1);
                motion_stop(&mot2, &st2);
                printf("GO(A): %ld\n", rev_cmd);
            }
            else if (line[0] == 'B' || line[0] == 'b')
            {
                long rev_cmd = strtol(line + 1, NULL, 10);
                g_sync_both = 0;
                motion_start_single(&mot2, &st2, rev_cmd, now, enc2_reset, 0);
                motion_stop(&mot1, &st1);
                printf("GO(B): %ld\n", rev_cmd);
            }
            else
            {
                long rev_cmd = strtol(line, NULL, 10);
                start_both(&mot1, &mot2, &st1, &st2, rev_cmd, now);
                printf("GO(BOTH): %ld\n", rev_cmd);
            }
        }

        if (g_sync_both) {
            sync_update(&mot1, &mot2, &st1, &st2, now);
        } else {
            motion_update_single(&mot1, &st1, now, enc1_get_count, "A", 1);
            motion_update_single(&mot2, &st2, now, enc2_get_count, "B", 0);
        }

        if ((now - last_print) >= PRINT_PERIOD_MS)
        {
            last_print = now;
            int32_t c1 = enc1_get_count();
            int32_t c2 = enc2_get_count();
            printf("sync=%u | runA=%u cntA=%ld | runB=%u cntB=%ld | d=%ld\n",
                   (unsigned)g_sync_both,
                   (unsigned)st1.running, (long)c1,
                   (unsigned)st2.running, (long)c2,
                   (long)(iabs32(c1) - iabs32(c2)));
        }
    }
}
