#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Thu_vien_L298.h"
#include "Motor_Encoder_Cua_Quan.h"

/* ================= UART ================= */
#define BAUDRATE 115200U

/* ================== ENCODER ================== */
#define COUNTS_PER_REV     1000L   /* nếu encoder khác nhau, hãy tách CPR riêng */

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

/* ================== SYNC (2+2) ==================
   - Cụm phải: A và B đồng bộ theo sai lệch tiến độ (P-coupling)
   - Cụm trái: LF và LS đồng bộ theo sai lệch tiến độ (P-coupling)
   - Cụm trái bám theo tiến độ cụm phải: progR=(A+B)/2
   - HƯỚNG B: điều kiện DỪNG theo progR (không dùng prog_min)
*/
#define SYNC_ENABLE        1

#define SYNC_ERR_DIV_AB       200
#define SYNC_ERR_DIV_LL       200   /* LF vs LS */
#define SYNC_ERR_DIV_L_FOLLOW 200   /* (LF+LS)/2 bám progR */

#define SYNC_MAX_CORR_8BIT    40

#define SYNC_MIN_8BIT         20
#define SYNC_MAX_8BIT         220

/* ================== CÁCH B: BIAS/GAIN ==================
   Gợi ý: để 100 trước, rồi tăng/giảm nhẹ. 200 sẽ rất dễ bão hòa duty.
*/
#define M1_GAIN_PCT        200   /* A: phải sau */
#define M2_GAIN_PCT        200   /* B: phải trước */
#define M3_GAIN_PCT        200   /* LF: trái trước */
#define M4_GAIN_PCT        200   /* LS: trái sau */

#define M1_BIAS_8BIT       0
#define M2_BIAS_8BIT       0
#define M3_BIAS_8BIT       0
#define M4_BIAS_8BIT       0

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
static inline int32_t imin32(int32_t a, int32_t b) { return (a < b) ? a : b; }

static uint8_t clamp_u8(int32_t x)
{
    if (x < 0) return 0;
    if (x > 255) return 255;
    return (uint8_t)x;
}

static int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Apply fixed calibration (CÁCH B) */
static uint8_t apply_calib(uint8_t duty, int32_t gain_pct, int32_t bias_8bit)
{
    int32_t d = (int32_t)duty;
    d = (d * gain_pct) / 100;
    d += bias_8bit;

    if (d < (int32_t)SYNC_MIN_8BIT) d = (int32_t)SYNC_MIN_8BIT;
    if (d > (int32_t)SYNC_MAX_8BIT) d = (int32_t)SYNC_MAX_8BIT;

    return clamp_u8(d);
}
static uint8_t apply_calib_m1(uint8_t duty) { return apply_calib(duty, (int32_t)M1_GAIN_PCT, (int32_t)M1_BIAS_8BIT); }
static uint8_t apply_calib_m2(uint8_t duty) { return apply_calib(duty, (int32_t)M2_GAIN_PCT, (int32_t)M2_BIAS_8BIT); }
static uint8_t apply_calib_m3(uint8_t duty) { return apply_calib(duty, (int32_t)M3_GAIN_PCT, (int32_t)M3_BIAS_8BIT); }
static uint8_t apply_calib_m4(uint8_t duty) { return apply_calib(duty, (int32_t)M4_GAIN_PCT, (int32_t)M4_BIAS_8BIT); }

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

/* ================== Encoder #1 (A: TIM3 PC6/PC7) ================== */
static EncoderEXTI_CQ_Handle g_enc1;
static int32_t encA_get_count(void) { return EncoderEXTI_CQ_GetCount(&g_enc1); }
static void    encA_reset(void)     { EncoderEXTI_CQ_Reset(&g_enc1); }

/* ================== Encoder #2 (B: EXTI PC9/PC8) ================== */
static volatile int32_t  g_encB_count32 = 0;
static volatile uint8_t  g_encB_lastAB  = 0;

static const int8_t QUAD_TABLE_B[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t encB_readA_pc9(void) { return (GPIOC->IDR & (1U << 9)) ? 1U : 0U; }
static inline uint8_t encB_readB_pc8(void) { return (GPIOC->IDR & (1U << 8)) ? 1U : 0U; }

static void encoderB_init_exti_pc8_pc9_pullup(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    GPIOC->MODER &= ~(3U << (2U * 8U));
    GPIOC->MODER &= ~(3U << (2U * 9U));

    GPIOC->PUPDR &= ~(3U << (2U * 8U));
    GPIOC->PUPDR &= ~(3U << (2U * 9U));
    GPIOC->PUPDR |=  (1U << (2U * 8U));
    GPIOC->PUPDR |=  (1U << (2U * 9U));

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    SYSCFG->EXTICR[2] &= ~((0xFU << 0) | (0xFU << 4));
    SYSCFG->EXTICR[2] |=  ((0x2U << 0) | (0x2U << 4)); /* GPIOC */

    EXTI->IMR  |= (1U << 8) | (1U << 9);
    EXTI->RTSR |= (1U << 8) | (1U << 9);
    EXTI->FTSR |= (1U << 8) | (1U << 9);
    EXTI->PR = (1U << 8) | (1U << 9);

    g_encB_lastAB = (uint8_t)((encB_readA_pc9() << 1) | encB_readB_pc8());

    NVIC_SetPriority(EXTI9_5_IRQn, 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;
    if (pr & ((1U << 8) | (1U << 9))) {
        EXTI->PR = (1U << 8) | (1U << 9);

        uint8_t a  = encB_readA_pc9();
        uint8_t b  = encB_readB_pc8();
        uint8_t ab = (uint8_t)((a << 1) | b);

        uint8_t idx = (uint8_t)((g_encB_lastAB << 2) | ab);
        g_encB_count32 += (int32_t)QUAD_TABLE_B[idx];
        g_encB_lastAB = ab;
    }
}

static int32_t encB_get_count(void)
{
    __disable_irq();
    int32_t c = g_encB_count32;
    __enable_irq();
    return c;
}

static void encB_reset(void)
{
    __disable_irq();
    g_encB_count32 = 0;
    g_encB_lastAB  = (uint8_t)((encB_readA_pc9() << 1) | encB_readB_pc8());
    __enable_irq();
}

/* ================== Encoder #3 (LF: TIM1 PA8/PA9) ================== */
static void encoderLF_init_tim1_pa8_pa9_pullup(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    /* PA8/PA9 AF1 (TIM1), pull-up */
    GPIOA->MODER &= ~(3U << (2U * 8U));
    GPIOA->MODER |=  (2U << (2U * 8U));
    GPIOA->MODER &= ~(3U << (2U * 9U));
    GPIOA->MODER |=  (2U << (2U * 9U));

    GPIOA->AFR[1] &= ~(0xFU << (4U * (8U - 8U)));
    GPIOA->AFR[1] |=  (1U   << (4U * (8U - 8U)));
    GPIOA->AFR[1] &= ~(0xFU << (4U * (9U - 8U)));
    GPIOA->AFR[1] |=  (1U   << (4U * (9U - 8U)));

    GPIOA->PUPDR &= ~(3U << (2U * 8U));
    GPIOA->PUPDR &= ~(3U << (2U * 9U));
    GPIOA->PUPDR |=  (1U << (2U * 8U));
    GPIOA->PUPDR |=  (1U << (2U * 9U));

    /* TIM1 encoder mode */
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    TIM1->SMCR = 0;
    TIM1->DIER = 0;

    TIM1->ARR = 0xFFFFU;
    TIM1->PSC = 0;

    TIM1->CCMR1 = 0;
    TIM1->CCMR1 |= (1U << 0);   /* CC1S=01 */
    TIM1->CCMR1 |= (1U << 8);   /* CC2S=01 */

    TIM1->CCER = 0;             /* rising */
    TIM1->SMCR |= (3U << 0);    /* SMS=011 encoder mode 3 */
    TIM1->CNT = 0;

    TIM1->CR1 |= TIM_CR1_CEN;
}

static struct {
    int32_t acc;
    uint16_t last;
} g_encLF = {0,0};

static int32_t encLF_get_count(void)
{
    uint16_t now = (uint16_t)TIM1->CNT;
    int16_t diff = (int16_t)(now - g_encLF.last);
    g_encLF.last = now;
    g_encLF.acc += (int32_t)diff;
    return g_encLF.acc;
}

static void encLF_reset(void)
{
    TIM1->CNT = 0;
    g_encLF.acc = 0;
    g_encLF.last = 0;
}

/* ================== Encoder #4 (LS: EXTI PC11/PC10) ==================
   C1=PC11, C2=PC10
   Dùng EXTI line 11 và 10, IRQ EXTI15_10_IRQn
*/
static volatile int32_t  g_encLS_count32 = 0;
static volatile uint8_t  g_encLS_lastAB  = 0;

static const int8_t QUAD_TABLE_LS[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t encLS_readA_pc11(void) { return (GPIOC->IDR & (1U << 11)) ? 1U : 0U; } /* C1 */
static inline uint8_t encLS_readB_pc10(void) { return (GPIOC->IDR & (1U << 10)) ? 1U : 0U; } /* C2 */

static void encoderLS_init_exti_pc10_pc11_pullup(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* PC10, PC11 input */
    GPIOC->MODER &= ~(3U << (2U * 10U));
    GPIOC->MODER &= ~(3U << (2U * 11U));

    /* pull-up */
    GPIOC->PUPDR &= ~(3U << (2U * 10U));
    GPIOC->PUPDR &= ~(3U << (2U * 11U));
    GPIOC->PUPDR |=  (1U << (2U * 10U));
    GPIOC->PUPDR |=  (1U << (2U * 11U));

    /* EXTI10/11 -> GPIOC (port C = 0x2) */
    SYSCFG->EXTICR[2] &= ~((0xFU << 8) | (0xFU << 12));
    SYSCFG->EXTICR[2] |=  ((0x2U << 8) | (0x2U << 12));

    EXTI->IMR  |= (1U << 10) | (1U << 11);
    EXTI->RTSR |= (1U << 10) | (1U << 11);
    EXTI->FTSR |= (1U << 10) | (1U << 11);
    EXTI->PR    = (1U << 10) | (1U << 11);

    g_encLS_lastAB = (uint8_t)((encLS_readA_pc11() << 1) | encLS_readB_pc10());

    NVIC_SetPriority(EXTI15_10_IRQn, 5);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;

    if (pr & ((1U << 10) | (1U << 11))) {
        EXTI->PR = (1U << 10) | (1U << 11);

        uint8_t a  = encLS_readA_pc11();
        uint8_t b  = encLS_readB_pc10();
        uint8_t ab = (uint8_t)((a << 1) | b);

        uint8_t idx = (uint8_t)((g_encLS_lastAB << 2) | ab);
        g_encLS_count32 += (int32_t)QUAD_TABLE_LS[idx];
        g_encLS_lastAB = ab;
    }
}

static int32_t encLS_get_count(void)
{
    __disable_irq();
    int32_t c = g_encLS_count32;
    __enable_irq();
    return c;
}

static void encLS_reset(void)
{
    __disable_irq();
    g_encLS_count32 = 0;
    g_encLS_lastAB  = (uint8_t)((encLS_readA_pc11() << 1) | encLS_readB_pc10());
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
                                uint8_t which) /* 1=A, 2=B, 3=LF, 4=LS */
{
    if (rev_cmd == 0) { motion_stop(mot, st); return; }

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
        if (which == 1) duty = apply_calib_m1(duty);
        else if (which == 2) duty = apply_calib_m2(duty);
        else if (which == 3) duty = apply_calib_m3(duty);
        else duty = apply_calib_m4(duty);

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
                                 uint8_t which)
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

    if (which == 1) base = apply_calib_m1(base);
    else if (which == 2) base = apply_calib_m2(base);
    else if (which == 3) base = apply_calib_m3(base);
    else base = apply_calib_m4(base);

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

/* ================== SYNC (2+2) ================== */
static uint8_t g_sync_all = 0;

static void start_all(L298_Handle_t *mA, L298_Handle_t *mB, L298_Handle_t *mLF, L298_Handle_t *mLS,
                      Motion_t *sA, Motion_t *sB, Motion_t *sLF, Motion_t *sLS,
                      long rev_cmd, uint32_t now)
{
    if (rev_cmd == 0) {
        motion_stop(mA, sA);
        motion_stop(mB, sB);
        motion_stop(mLF, sLF);
        motion_stop(mLS, sLS);
        g_sync_all = 0;
        return;
    }

    encA_reset();
    encB_reset();
    encLF_reset();
    encLS_reset();

    long ra = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
    int32_t target = (int32_t)(ra * (long)COUNTS_PER_REV);

    sA->cmd_revs_signed  = rev_cmd;
    sB->cmd_revs_signed  = rev_cmd;
    sLF->cmd_revs_signed = rev_cmd;
    sLS->cmd_revs_signed = rev_cmd;

    sA->target_counts  = target;
    sB->target_counts  = target;
    sLF->target_counts = target;
    sLS->target_counts = target;

    sA->last_cnt = 0; sB->last_cnt = 0; sLF->last_cnt = 0; sLS->last_cnt = 0;
    sA->last_move_ms = now; sB->last_move_ms = now; sLF->last_move_ms = now; sLS->last_move_ms = now;

    if (rev_cmd > 0) {
        L298_SetDir(mA,  L298_DIR_FWD);
        L298_SetDir(mB,  L298_DIR_FWD);
        L298_SetDir(mLF, L298_DIR_FWD);
        L298_SetDir(mLS, L298_DIR_FWD);
    } else {
        L298_SetDir(mA,  L298_DIR_REV);
        L298_SetDir(mB,  L298_DIR_REV);
        L298_SetDir(mLF, L298_DIR_REV);
        L298_SetDir(mLS, L298_DIR_REV);
    }

    L298_SetSpeed8(mA,  apply_calib_m1(SPEED_KICK_8BIT));
    L298_SetSpeed8(mB,  apply_calib_m2(SPEED_KICK_8BIT));
    L298_SetSpeed8(mLF, apply_calib_m3(SPEED_KICK_8BIT));
    L298_SetSpeed8(mLS, apply_calib_m4(SPEED_KICK_8BIT));

    L298_Enable(mA,  1);
    L298_Enable(mB,  1);
    L298_Enable(mLF, 1);
    L298_Enable(mLS, 1);

    sA->running  = 1;
    sB->running  = 1;
    sLF->running = 1;
    sLS->running = 1;

    sA->run_start_ms  = now;
    sB->run_start_ms  = now;
    sLF->run_start_ms = now;
    sLS->run_start_ms = now;

    g_sync_all = 1;
}

static void sync_update_all(L298_Handle_t *mA, L298_Handle_t *mB, L298_Handle_t *mLF, L298_Handle_t *mLS,
                            Motion_t *sA, Motion_t *sB, Motion_t *sLF, Motion_t *sLS,
                            uint32_t now)
{
#if !SYNC_ENABLE
    motion_update_single(mA,  sA,  now, encA_get_count,  "A",  1);
    motion_update_single(mB,  sB,  now, encB_get_count,  "B",  2);
    motion_update_single(mLF, sLF, now, encLF_get_count, "LF", 3);
    motion_update_single(mLS, sLS, now, encLS_get_count, "LS", 4);
    return;
#endif

    if (!sA->running || !sB->running || !sLF->running || !sLS->running) { g_sync_all = 0; return; }

    int32_t cA  = encA_get_count();
    int32_t cB  = encB_get_count();
    int32_t cLF = encLF_get_count();
    int32_t cLS = encLS_get_count();

    int32_t magA  = iabs32(cA);
    int32_t magB  = iabs32(cB);
    int32_t magLF = iabs32(cLF);
    int32_t magLS = iabs32(cLS);

    /* no-move detection (giữ nguyên) */
    if (cA  != sA->last_cnt)  { sA->last_cnt  = cA;  sA->last_move_ms  = now; }
    if (cB  != sB->last_cnt)  { sB->last_cnt  = cB;  sB->last_move_ms  = now; }
    if (cLF != sLF->last_cnt) { sLF->last_cnt = cLF; sLF->last_move_ms = now; }
    if (cLS != sLS->last_cnt) { sLS->last_cnt = cLS; sLS->last_move_ms = now; }

    if ((now - sA->last_move_ms)  >= NO_MOVE_RESET_MS ||
        (now - sB->last_move_ms)  >= NO_MOVE_RESET_MS ||
        (now - sLF->last_move_ms) >= NO_MOVE_RESET_MS ||
        (now - sLS->last_move_ms) >= NO_MOVE_RESET_MS) {
        motion_stop(mA,  sA);
        motion_stop(mB,  sB);
        motion_stop(mLF, sLF);
        motion_stop(mLS, sLS);
        g_sync_all = 0;
        printf("WARN(SYNC): %ums khong thay xung -> stop all\n", (unsigned)NO_MOVE_RESET_MS);
        return;
    }

    uint32_t run_age = now - sA->run_start_ms;

    /* ===== mốc tiến độ cụm phải ===== */
    int32_t progR = (magA + magB) / 2;
    int32_t remain = sA->target_counts - progR;
    if (remain < 0) remain = 0;

    uint8_t base;
    if (run_age < KICK_MS) base = SPEED_KICK_8BIT;
    else {
        int32_t ramp_counts  = (int32_t)(SLOW_RAMP_REVS * (long)COUNTS_PER_REV);
        int32_t final_counts = (int32_t)((FINAL_REVS_NUM * (long)COUNTS_PER_REV) / FINAL_REVS_DEN);
        base = speed_from_remain(remain, ramp_counts, final_counts);
    }

    /* ===== AB coupling ===== */
    int32_t errAB  = magA - magB; /* >0: A đi trước */
    int32_t corrAB = errAB / SYNC_ERR_DIV_AB;
    corrAB = clamp_i32(corrAB, -(int32_t)SYNC_MAX_CORR_8BIT, (int32_t)SYNC_MAX_CORR_8BIT);

    int32_t dA = (int32_t)base - corrAB;
    int32_t dB = (int32_t)base + corrAB;

    /* ===== Left follow + left pair coupling ===== */
    int32_t progL = (magLF + magLS) / 2;

    /* group follow: left bám progR */
    int32_t errLFol  = progL - progR; /* >0: left đi trước right */
    int32_t corrLFol = errLFol / SYNC_ERR_DIV_L_FOLLOW;
    corrLFol = clamp_i32(corrLFol, -(int32_t)SYNC_MAX_CORR_8BIT, (int32_t)SYNC_MAX_CORR_8BIT);

    int32_t baseL = (int32_t)base - corrLFol; /* left ahead -> giảm duty; left chậm -> tăng */

    /* LF vs LS pair coupling */
    int32_t errLL  = magLF - magLS; /* >0: LF đi trước LS */
    int32_t corrLL = errLL / SYNC_ERR_DIV_LL;
    corrLL = clamp_i32(corrLL, -(int32_t)SYNC_MAX_CORR_8BIT, (int32_t)SYNC_MAX_CORR_8BIT);

    int32_t dLF = baseL - corrLL;
    int32_t dLS = baseL + corrLL;

    /* apply calibration after coupling */
    uint8_t outA  = apply_calib_m1(clamp_u8(dA));
    uint8_t outB  = apply_calib_m2(clamp_u8(dB));
    uint8_t outLF = apply_calib_m3(clamp_u8(dLF));
    uint8_t outLS = apply_calib_m4(clamp_u8(dLS));

    L298_SetSpeed8(mA,  outA);
    L298_SetSpeed8(mB,  outB);
    L298_SetSpeed8(mLF, outLF);
    L298_SetSpeed8(mLS, outLS);

    /* ===== stop condition: HƯỚNG B - dừng theo progR (cụm phải) ===== */
    long cmd_abs = (sA->cmd_revs_signed > 0) ? sA->cmd_revs_signed : -sA->cmd_revs_signed;

    int32_t stop_margin = 0;
    if (cmd_abs >= BIG_CMD_REVS_THRESHOLD) stop_margin = (int32_t)(STOP_MARGIN_REVS_BIG   * (long)COUNTS_PER_REV);
    else                                  stop_margin = (int32_t)(STOP_MARGIN_REVS_SMALL * (long)COUNTS_PER_REV);

    int32_t stop_at = sA->target_counts - stop_margin;
    if (stop_at < 0) stop_at = 0;

    if (run_age >= MIN_RUN_MS && progR >= stop_at)
    {
        sA->running = 0; sB->running = 0; sLF->running = 0; sLS->running = 0;
        g_sync_all = 0;

        uint32_t brake_ms = 0;
        if (cmd_abs <= SMALL_CMD_REVS_MAX) {
            brake_ms = BRAKE_MS_SMALL;
            L298_Coast(mA);  L298_Enable(mA,  0);
            L298_Coast(mB);  L298_Enable(mB,  0);
            L298_Coast(mLF); L298_Enable(mLF, 0);
            L298_Coast(mLS); L298_Enable(mLS, 0);
        } else {
            brake_ms = BRAKE_MS_BIG;

            L298_SetDutyPercent(mA, 100);  L298_Enable(mA, 1);  L298_Brake(mA);
            L298_SetDutyPercent(mB, 100);  L298_Enable(mB, 1);  L298_Brake(mB);
            L298_SetDutyPercent(mLF,100);  L298_Enable(mLF,1);  L298_Brake(mLF);
            L298_SetDutyPercent(mLS,100);  L298_Enable(mLS,1);  L298_Brake(mLS);

            uint32_t t0 = millis_ms();
            while ((millis_ms() - t0) < brake_ms) {
                uint32_t t = millis_ms();
                L298_Task(mA, t);
                L298_Task(mB, t);
                L298_Task(mLF,t);
                L298_Task(mLS,t);
            }

            L298_Coast(mA);  L298_Enable(mA,  0);
            L298_Coast(mB);  L298_Enable(mB,  0);
            L298_Coast(mLF); L298_Enable(mLF, 0);
            L298_Coast(mLS); L298_Enable(mLS, 0);
        }

        printf("DONE(SYNC 2+2, stop=progR): progR=%ld progL=%ld | cA=%ld cB=%ld cLF=%ld cLS=%ld | dAB=%ld dLL=%ld dLfol=%ld | brake=%ums\n",
               (long)progR, (long)progL,
               (long)cA, (long)cB, (long)cLF, (long)cLS,
               (long)(magA - magB),
               (long)(magLF - magLS),
               (long)(progL - progR),
               (unsigned)brake_ms);
    }
}

/* ================== main ================== */
int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    usart2_init_tx_rx();

    /* Encoder A: TIM3 PC6/PC7 */
    EncoderEXTI_CQ_Init(&g_enc1);

    /* Encoder B: EXTI PC8/PC9 */
    encoderB_init_exti_pc8_pc9_pullup();

    /* Encoder LF: TIM1 PA8/PA9 */
    encoderLF_init_tim1_pa8_pa9_pullup();
    encLF_reset();

    /* Encoder LS: EXTI PC10/PC11 */
    encoderLS_init_exti_pc10_pc11_pullup();
    encLS_reset();

    /* Motor A (M1): phải sau  EN=PD11, IN=PD13/PD14 */
    L298_Pins_t pins_mA = {
        .ENA_Port = GPIOD, .ENA_Pin = 11,
        .IN1_Port = GPIOD, .IN1_Pin = 13,
        .IN2_Port = GPIOD, .IN2_Pin = 14
    };
    L298_Handle_t motA;
    L298_Init(&motA, &pins_mA, PWM_PERIOD_MS);

    /* Motor B (M2): phải trước EN=PD12, IN=PD9/PD10 */
    L298_Pins_t pins_mB = {
        .ENA_Port = GPIOD, .ENA_Pin = 12,
        .IN1_Port = GPIOD, .IN1_Pin = 9,
        .IN2_Port = GPIOD, .IN2_Pin = 10
    };
    L298_Handle_t motB;
    L298_Init(&motB, &pins_mB, PWM_PERIOD_MS);

    /* Motor LF (M3): trái trước EN=PB12, IN=PB13, IN=PB14 */
    L298_Pins_t pins_mLF = {
        .ENA_Port = GPIOB, .ENA_Pin = 12,
        .IN1_Port = GPIOB, .IN1_Pin = 13,
        .IN2_Port = GPIOB, .IN2_Pin = 14
    };
    L298_Handle_t motLF;
    L298_Init(&motLF, &pins_mLF, PWM_PERIOD_MS);

    /* Motor LS (M4): trái sau ENA=PB11, IN1=PB10, IN2=PE15 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;

    L298_Pins_t pins_mLS = {
        .ENA_Port = GPIOB, .ENA_Pin = 11,
        .IN1_Port = GPIOB, .IN1_Pin = 10,
        .IN2_Port = GPIOE, .IN2_Pin = 15
    };
    L298_Handle_t motLS;
    L298_Init(&motLS, &pins_mLS, PWM_PERIOD_MS);

#if AUTO_TEST_ENABLE
    if (AUTO_TEST_DIR_FWD) {
        L298_SetDir(&motA,  L298_DIR_FWD);
        L298_SetDir(&motB,  L298_DIR_FWD);
        L298_SetDir(&motLF, L298_DIR_FWD);
        L298_SetDir(&motLS, L298_DIR_FWD);
    } else {
        L298_SetDir(&motA,  L298_DIR_REV);
        L298_SetDir(&motB,  L298_DIR_REV);
        L298_SetDir(&motLF, L298_DIR_REV);
        L298_SetDir(&motLS, L298_DIR_REV);
    }

    L298_SetSpeed8(&motA,  apply_calib_m1(AUTO_TEST_SPEED8));
    L298_SetSpeed8(&motB,  apply_calib_m2(AUTO_TEST_SPEED8));
    L298_SetSpeed8(&motLF, apply_calib_m3(AUTO_TEST_SPEED8));
    L298_SetSpeed8(&motLS, apply_calib_m4(AUTO_TEST_SPEED8));

    L298_Enable(&motA,  1);
    L298_Enable(&motB,  1);
    L298_Enable(&motLF, 1);
    L298_Enable(&motLS, 1);

    uint32_t t0 = millis_ms();
    while ((millis_ms() - t0) < AUTO_TEST_MS) {
        uint32_t t = millis_ms();
        EncoderEXTI_CQ_Task(&g_enc1);
        (void)encLF_get_count();
        L298_Task(&motA,  t);
        L298_Task(&motB,  t);
        L298_Task(&motLF, t);
        L298_Task(&motLS, t);
    }

    L298_Coast(&motA);  L298_Enable(&motA,  0);
    L298_Coast(&motB);  L298_Enable(&motB,  0);
    L298_Coast(&motLF); L298_Enable(&motLF, 0);
    L298_Coast(&motLS); L298_Enable(&motLS, 0);
#endif

    printf("START:\n");
    printf("  A(M1)  [PHAI SAU] : EN=PD11 IN=PD13/PD14 | ENC_A : TIM3 PC6/PC7\n");
    printf("  B(M2)  [PHAI TRUOC]:EN=PD12 IN=PD9/PD10  | ENC_B : EXTI PC9/PC8\n");
    printf("  LF(M3) [TRAI TRUOC]:EN=PB12 IN=PB13/PB14 | ENC_LF: TIM1 PA8/PA9\n");
    printf("  LS(M4) [TRAI SAU] : EN=PB11 IN1=PB10 IN2=PE15 | ENC_LS: EXTI PC11/PC10\n");

    printf("SYNC: stop theo progR=(|A|+|B|)/2 (HUONG B)\n");

    printf("CMD:\n");
    printf("  <n>    : chay dong bo 4 banh n vong (am: nguoc)\n");
    printf("  A<n>   : chi banh A\n");
    printf("  B<n>   : chi banh B\n");
    printf("  L<n>   : chi banh LF (trai truoc)\n");
    printf("  S<n>   : chi banh LS (trai sau)\n");
    printf("  0      : stop\n");
    printf("  RST    : reset 4 encoder\n");

    Motion_t stA = {0}, stB = {0}, stLF = {0}, stLS = {0};
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
        (void)encLF_get_count();

        L298_Task(&motA,  now);
        L298_Task(&motB,  now);
        L298_Task(&motLF, now);
        L298_Task(&motLS, now);

        if (cmd_poll_line(line, sizeof(line)))
        {
            if (strcmp(line, "RST") == 0 || strcmp(line, "rst") == 0)
            {
                encA_reset();
                encB_reset();
                encLF_reset();
                encLS_reset();

                motion_stop(&motA,  &stA);
                motion_stop(&motB,  &stB);
                motion_stop(&motLF, &stLF);
                motion_stop(&motLS, &stLS);

                g_sync_all = 0;
                printf("OK: reset encA+encB+encLF+encLS, stop all\n");
            }
            else if (strcmp(line, "0") == 0)
            {
                motion_stop(&motA,  &stA);
                motion_stop(&motB,  &stB);
                motion_stop(&motLF, &stLF);
                motion_stop(&motLS, &stLS);
                g_sync_all = 0;
                printf("STOP ALL\n");
            }
            else if (line[0] == 'A' || line[0] == 'a')
            {
                long rev_cmd = strtol(line + 1, NULL, 10);
                g_sync_all = 0;
                motion_start_single(&motA, &stA, rev_cmd, now, encA_reset, 1);
                motion_stop(&motB,  &stB);
                motion_stop(&motLF, &stLF);
                motion_stop(&motLS, &stLS);
                printf("GO(A): %ld\n", rev_cmd);
            }
            else if (line[0] == 'B' || line[0] == 'b')
            {
                long rev_cmd = strtol(line + 1, NULL, 10);
                g_sync_all = 0;
                motion_start_single(&motB, &stB, rev_cmd, now, encB_reset, 2);
                motion_stop(&motA,  &stA);
                motion_stop(&motLF, &stLF);
                motion_stop(&motLS, &stLS);
                printf("GO(B): %ld\n", rev_cmd);
            }
            else if (line[0] == 'L' || line[0] == 'l')
            {
                long rev_cmd = strtol(line + 1, NULL, 10);
                g_sync_all = 0;
                motion_start_single(&motLF, &stLF, rev_cmd, now, encLF_reset, 3);
                motion_stop(&motA,  &stA);
                motion_stop(&motB,  &stB);
                motion_stop(&motLS, &stLS);
                printf("GO(LF): %ld\n", rev_cmd);
            }
            else if (line[0] == 'S' || line[0] == 's')
            {
                long rev_cmd = strtol(line + 1, NULL, 10);
                g_sync_all = 0;
                motion_start_single(&motLS, &stLS, rev_cmd, now, encLS_reset, 4);
                motion_stop(&motA,  &stA);
                motion_stop(&motB,  &stB);
                motion_stop(&motLF, &stLF);
                printf("GO(LS): %ld\n", rev_cmd);
            }
            else
            {
                long rev_cmd = strtol(line, NULL, 10);
                start_all(&motA, &motB, &motLF, &motLS, &stA, &stB, &stLF, &stLS, rev_cmd, now);
                printf("GO(ALL SYNC 2+2, stop=progR): %ld\n", rev_cmd);
            }
        }

        if (g_sync_all) {
            sync_update_all(&motA, &motB, &motLF, &motLS, &stA, &stB, &stLF, &stLS, now);
        } else {
            motion_update_single(&motA,  &stA,  now, encA_get_count,  "A",  1);
            motion_update_single(&motB,  &stB,  now, encB_get_count,  "B",  2);
            motion_update_single(&motLF, &stLF, now, encLF_get_count, "LF", 3);
            motion_update_single(&motLS, &stLS, now, encLS_get_count, "LS", 4);
        }

        if ((now - last_print) >= PRINT_PERIOD_MS)
        {
            last_print = now;

            int32_t cA  = encA_get_count();
            int32_t cB  = encB_get_count();
            int32_t cLF = encLF_get_count();
            int32_t cLS = encLS_get_count();

            int32_t magA  = iabs32(cA);
            int32_t magB  = iabs32(cB);
            int32_t magLF = iabs32(cLF);
            int32_t magLS = iabs32(cLS);

            int32_t progR = (magA + magB) / 2;
            int32_t progL = (magLF + magLS) / 2;

            printf("sync=%u | runA=%u cA=%ld | runB=%u cB=%ld | runLF=%u cLF=%ld | runLS=%u cLS=%ld | progR=%ld progL=%ld | dAB=%ld dLL=%ld dLfol=%ld\n",
                   (unsigned)g_sync_all,
                   (unsigned)stA.running,  (long)cA,
                   (unsigned)stB.running,  (long)cB,
                   (unsigned)stLF.running, (long)cLF,
                   (unsigned)stLS.running, (long)cLS,
                   (long)progR, (long)progL,
                   (long)(magA - magB),
                   (long)(magLF - magLS),
                   (long)(progL - progR));
        }
    }
}
