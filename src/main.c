#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Thu_vien_L298.h"
#include "Motor_Encoder_Cua_Quan.h"

/* ================= UART ================= */
#define BAUDRATE 115200U

/* ================== ENCODER ================== */
#define COUNTS_PER_REV     1000L   /* giữ như bạn đang dùng */

/* ================== L298 Soft PWM ================== */
#define PWM_PERIOD_MS      5U

/* Kick + min-run */
#define SPEED_KICK_8BIT    140U
#define KICK_MS            180U
#define MIN_RUN_MS         120U

/* Speed limits */
#define SPEED_FAST_8BIT     65U
#define SPEED_SLOW_8BIT     35U

/* Brake: lớn/nhỏ tách riêng */
#define BRAKE_MS_BIG       120U
#define BRAKE_MS_SMALL      0U   /* LỆNH NHỎ: không phanh cưỡng bức để tránh giật ngược */

/* Ramp vùng giảm tốc */
#define SLOW_RAMP_REVS      8L     /* tăng ramp để giảm quán tính */
#define FINAL_REVS_NUM      1L
#define FINAL_REVS_DEN      5L

/* Stop margin (dừng sớm) */
#define BIG_CMD_REVS_THRESHOLD 15L  /* >15 vòng xem là lệnh lớn */
#define STOP_MARGIN_REVS_BIG    1L  /* giảm xuống 1 vòng (2 vòng có thể gây thiếu mạnh) */
#define STOP_MARGIN_REVS_SMALL  0L

/* Nhóm “lệnh nhỏ” để tắt BRAKE hoàn toàn */
#define SMALL_CMD_REVS_MAX      5L

#define PRINT_PERIOD_MS    500U
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
                ( (int32_t)(SPEED_FAST_8BIT - SPEED_SLOW_8BIT) * num ) / den;

    if (s < (int32_t)SPEED_SLOW_8BIT) s = SPEED_SLOW_8BIT;
    if (s > (int32_t)SPEED_FAST_8BIT) s = SPEED_FAST_8BIT;
    return (uint8_t)s;
}

/* ================== Encoder handle ================== */
static EncoderEXTI_CQ_Handle g_enc;

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    usart2_init_tx_rx();
    EncoderEXTI_CQ_Init(&g_enc);

    L298_Pins_t pins = {
        .ENA_Port = GPIOC, .ENA_Pin = 8,
        .IN1_Port = GPIOC, .IN1_Pin = 13,
        .IN2_Port = GPIOE, .IN2_Pin = 5
    };
    L298_Handle_t mot;
    L298_Init(&mot, &pins, PWM_PERIOD_MS);

    printf("START: L298(PC8/PC13/PE5) + Encoder TIM3(PC6/PC7)\n");
    printf("COUNTS_PER_REV=%ld\n", (long)COUNTS_PER_REV);
    printf("KICK=%u/255 %ums | MIN_RUN=%ums | FAST=%u | SLOW=%u\n",
           (unsigned)SPEED_KICK_8BIT, (unsigned)KICK_MS,
           (unsigned)MIN_RUN_MS, (unsigned)SPEED_FAST_8BIT, (unsigned)SPEED_SLOW_8BIT);
    printf("RAMP=%ld revs | FINAL=%ld/%ld rev\n",
           (long)SLOW_RAMP_REVS, (long)FINAL_REVS_NUM, (long)FINAL_REVS_DEN);
    printf("BRAKE_BIG=%ums | BRAKE_SMALL=%ums | SMALL<=%ld rev\n",
           (unsigned)BRAKE_MS_BIG, (unsigned)BRAKE_MS_SMALL, (long)SMALL_CMD_REVS_MAX);
    printf("STOP_MARGIN_BIG=%ld rev | THRESH=%ld rev\n",
           (long)STOP_MARGIN_REVS_BIG, (long)BIG_CMD_REVS_THRESHOLD);

    uint8_t running = 0;
    int32_t target_counts = 0;
    long cmd_revs_signed = 0;
    uint32_t run_start_ms = 0;

    uint32_t last_print = millis_ms();
    int32_t last_cnt = EncoderEXTI_CQ_GetCount(&g_enc);
    uint32_t last_move_ms = millis_ms();

    char line[CMD_BUF_SZ];

    while (1)
    {
        uint32_t now = millis_ms();

        EncoderEXTI_CQ_Task(&g_enc);
        L298_Task(&mot, now);

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
                    L298_Enable(&mot, 0);
                    L298_Coast(&mot);
                    printf("STOP\n");
                }
                else
                {
                    cmd_revs_signed = rev_cmd;

                    long rev_abs = (rev_cmd > 0) ? rev_cmd : -rev_cmd;
                    target_counts = (int32_t)(rev_abs * (long)COUNTS_PER_REV);

                    EncoderEXTI_CQ_Reset(&g_enc);
                    last_cnt = 0;
                    last_move_ms = now;

                    if (rev_cmd > 0) L298_SetDir(&mot, L298_DIR_FWD);
                    else             L298_SetDir(&mot, L298_DIR_REV);

                    L298_SetSpeed8(&mot, SPEED_KICK_8BIT);
                    L298_Enable(&mot, 1);

                    running = 1;
                    run_start_ms = now;

                    printf("GO: rev=%ld -> target_counts=%ld\n", rev_cmd, (long)target_counts);
                }
            }
        }

        if (running)
        {
            int32_t cnt = EncoderEXTI_CQ_GetCount(&g_enc);
            int32_t mag = iabs32(cnt);

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

            if (running)
            {
                int32_t remain = target_counts - mag;
                if (remain < 0) remain = 0;

                uint32_t run_age = now - run_start_ms;

                if (run_age < KICK_MS) {
                    L298_SetSpeed8(&mot, SPEED_KICK_8BIT);
                } else {
                    int32_t ramp_counts  = (int32_t)(SLOW_RAMP_REVS * (long)COUNTS_PER_REV);
                    int32_t final_counts = (int32_t)((FINAL_REVS_NUM * (long)COUNTS_PER_REV) / FINAL_REVS_DEN);
                    uint8_t sp = speed_from_remain(remain, ramp_counts, final_counts);
                    L298_SetSpeed8(&mot, sp);
                }

                long cmd_revs_abs = (cmd_revs_signed > 0) ? cmd_revs_signed : -cmd_revs_signed;

                /* stop margin */
                int32_t stop_margin = 0;
                if (cmd_revs_abs >= BIG_CMD_REVS_THRESHOLD) {
                    stop_margin = (int32_t)(STOP_MARGIN_REVS_BIG * (long)COUNTS_PER_REV);
                } else {
                    stop_margin = (int32_t)(STOP_MARGIN_REVS_SMALL * (long)COUNTS_PER_REV);
                }

                int32_t stop_at = target_counts - stop_margin;
                if (stop_at < 0) stop_at = 0;

                if (run_age >= MIN_RUN_MS && mag >= stop_at)
                {
                    running = 0;

                    /* Ghi nhận ngay tại thời điểm vào dừng */
                    int32_t stop_cnt = EncoderEXTI_CQ_GetCount(&g_enc);
                    int32_t stop_mag = iabs32(stop_cnt);

                    /* Quy tắc: lệnh nhỏ -> không brake để tránh giật ngược */
                    uint32_t brake_ms = 0;
                    if (cmd_revs_abs <= SMALL_CMD_REVS_MAX) {
                        brake_ms = BRAKE_MS_SMALL; /* hiện =0 */
                        L298_Coast(&mot);
                        L298_Enable(&mot, 0);
                    } else {
                        brake_ms = BRAKE_MS_BIG;
                        L298_SetDutyPercent(&mot, 100);
                        L298_Enable(&mot, 1);
                        L298_Brake(&mot);

                        uint32_t t0 = millis_ms();
                        while ((millis_ms() - t0) < brake_ms) {
                            EncoderEXTI_CQ_Task(&g_enc);
                            L298_Task(&mot, millis_ms());
                        }

                        L298_Coast(&mot);
                        L298_Enable(&mot, 0);
                    }

                    /* Đọc lại sau pha dừng/phanh */
                    int32_t final_cnt = EncoderEXTI_CQ_GetCount(&g_enc);
                    int32_t final_mag = iabs32(final_cnt);

                    int32_t delta = (int32_t)final_mag - (int32_t)stop_mag; /* >0: trôi thêm, <0: giật ngược */
                    float rev_done = (float)final_mag / (float)COUNTS_PER_REV;

                    printf("DONE: final=%ld (~%.3f vong) | stop=%ld | delta=%ld | stop_at=%ld | margin=%ld | brake=%ums\n",
                           (long)final_mag, rev_done,
                           (long)stop_mag, (long)delta,
                           (long)stop_at, (long)stop_margin, (unsigned)brake_ms);
                }
            }
        }
        else
        {
            L298_Enable(&mot, 0);
        }

        if ((now - last_print) >= PRINT_PERIOD_MS)
        {
            last_print = now;

            int32_t c = EncoderEXTI_CQ_GetCount(&g_enc);
            int32_t a = iabs32(c);

            float rev = (float)a / (float)COUNTS_PER_REV;

            printf("run=%u | cnt=%ld (abs=%ld) | rev=%.3f\n",
                   (unsigned)running, (long)c, (long)a, rev);
        }
    }
}
