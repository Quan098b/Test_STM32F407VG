#include "stm32f4xx.h"
#include <stdint.h>

#define BAUDRATE 9600U   // phải trùng với BAUD_STM ở ESP

/* ---------- Lấy PCLK1 cho UART4 ---------- */
static uint32_t get_pclk1_hz(void)
{
    SystemCoreClockUpdate();
    static const uint8_t apb_tbl[8] = {1,1,1,1,2,4,8,16};
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;
    return SystemCoreClock / apb_tbl[ppre1];
}

/* ---------- UART4: PA0(TX), PA1(RX), AF8 ---------- */
static void uart4_init_pa0_tx_pa1_rx(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;

    /* PA0 = UART4_TX AF8 */
    GPIOA->MODER &= ~(3U << (2U * 0U));
    GPIOA->MODER |=  (2U << (2U * 0U));
    GPIOA->AFR[0] &= ~(0xFU << (4U * 0U));
    GPIOA->AFR[0] |=  (8U   << (4U * 0U));

    /* PA1 = UART4_RX AF8 */
    GPIOA->MODER &= ~(3U << (2U * 1U));
    GPIOA->MODER |=  (2U << (2U * 1U));
    GPIOA->AFR[0] &= ~(0xFU << (4U * 1U));
    GPIOA->AFR[0] |=  (8U   << (4U * 1U));

    UART4->CR1 = 0;
    UART4->CR2 = 0;
    UART4->CR3 = 0;

    UART4->BRR = (uint16_t)(get_pclk1_hz() / BAUDRATE);

    UART4->CR1 |= USART_CR1_TE | USART_CR1_RE;  // TE, RE dùng chung bit với USART
    UART4->CR1 |= USART_CR1_UE;
}

static void uart4_write_byte(uint8_t b)
{
    while (!(UART4->SR & USART_SR_TXE)) { }
    UART4->DR = b;
}

static void uart4_write_str(const char *s)
{
    while (*s) uart4_write_byte((uint8_t)*s++);
}

static int uart4_read_byte_nonblock(void)
{
    if (UART4->SR & USART_SR_RXNE) return (int)(uint8_t)UART4->DR;
    return -1;
}

int main(void)
{
    SystemCoreClockUpdate();
    uart4_init_pa0_tx_pa1_rx();

    uart4_write_str("STM32F4 UART4 PA0(TX)/PA1(RX) TEST\r\n");

    while (1)
    {
        /* Gửi chuỗi test liên tục */
        uart4_write_str("Quan0o\r\n");

        /* trễ thô ~ vài trăm ms (tuỳ F_CPU), bỏ SysTick cho chắc chắn */
        for (volatile uint32_t i = 0; i < 500000; ++i) {
            __NOP();
        }

        /* Echo lại mọi byte nhận được (nếu ESP gửi về) */
        int ch;
        while ((ch = uart4_read_byte_nonblock()) >= 0) {
            uart4_write_byte((uint8_t)ch);
        }
    }
}
