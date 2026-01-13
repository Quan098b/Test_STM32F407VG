#include "stm32f4xx.h"
#include <stdio.h>
#include "Motor_Encoder_Cua_Quan.h"

/* ===== UART printf (USART2 TX PA2) ===== */
#define BAUDRATE 115200U

/* ===== Encoder: đổi count -> số vòng ===== */
#define ENCODER_PPR        600U   // SỬA theo encoder của bạn (PPR 1 kênh)
#define ENCODER_QUAD_MULT  4U     // đang giải mã quadrature x4
#define COUNTS_PER_REV     ((int32_t)(ENCODER_PPR * ENCODER_QUAD_MULT))

static uint32_t get_pclk1_hz(void)
{
    SystemCoreClockUpdate();
    static const uint8_t apb_tbl[8] = {1,1,1,1,2,4,8,16};
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;
    return SystemCoreClock / apb_tbl[ppre1];
}

static void usart2_init_tx_only(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = AF7 (USART2_TX) */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));

    /* AF7 cho PA2 */
    GPIOA->AFR[0] &= ~(0xFU << (2U * 4U));
    GPIOA->AFR[0] |=  (7U   << (2U * 4U));

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    USART2->BRR = (uint16_t)(get_pclk1_hz() / BAUDRATE);

    USART2->CR1 |= USART_CR1_TE;
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

/* ===== Delay thô theo chu kỳ CPU ===== */
static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles--) { __NOP(); }
}

/* ===== Global handle để ISR dùng ===== */
static EncoderEXTI_CQ_Handle g_enc;

/* PC7 và PC9 thuộc EXTI9_5 */
void EXTI9_5_IRQHandler(void)
{
    EncoderEXTI_CQ_IRQHandler(&g_enc);
}

int main(void)
{
    usart2_init_tx_only();

    /* Encoder: C1=PC7, C2=PC9 */
    g_enc.gpioA = GPIOC; g_enc.pinA = 7;
    g_enc.gpioB = GPIOC; g_enc.pinB = 9;
    EncoderEXTI_CQ_Init(&g_enc);

    /* Motor L298: ENA=PC15, IN1=PC13, IN2=PE5 */
    MotorL298_CQ_Handle mot = {
        .gpio_ena = GPIOC, .pin_ena = 15,
        .gpio_in1 = GPIOC, .pin_in1 = 13,
        .gpio_in2 = GPIOE, .pin_in2 = 5
    };
    MotorL298_CQ_Init(&mot);

    /* Chạy thuận + bật ENA (ON/OFF) */
    MotorL298_CQ_SetDir(&mot, +1);
    MotorL298_CQ_Enable(&mot, 1);

    while (1)
    {
        /* Đếm tổng count, lấy trị tuyệt đối để "không tính chiều" */
        int32_t c = EncoderEXTI_CQ_GetCount(&g_enc);
        if (c < 0) c = -c;

        /* Tính số vòng nguyên + phần lẻ theo count */
        int32_t turns = c / COUNTS_PER_REV;  // số vòng nguyên
        int32_t rem   = c % COUNTS_PER_REV;  // phần lẻ 0..(COUNTS_PER_REV-1)

        printf("Da quay: %ld vong + %ld/%ld vong (cnt=%ld)\n",
               (long)turns, (long)rem, (long)COUNTS_PER_REV, (long)c);

        /* Giảm tốc độ in (chậm hơn 2 lần so với 4,000,000) */
        delay_cycles(8000000U);
    }
}
