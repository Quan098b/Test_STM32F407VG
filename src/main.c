#include "stm32f4xx.h"
#include "Led_Cua_Quan.h"
#include <stdio.h>

#define BAUDRATE 115200U

static uint32_t get_pclk1_hz(void)
{
    // Cập nhật SystemCoreClock theo cấu hình RCC hiện tại
    SystemCoreClockUpdate();

    // Tính APB1 prescaler từ RCC->CFGR (PPRE1 bits [12:10])
    static const uint8_t apb_presc_tbl[8] = {1,1,1,1,2,4,8,16};
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;

    return SystemCoreClock / apb_presc_tbl[ppre1];
}

static void usart2_init_tx_only(void)
{
    // USART2 nằm trên APB1, TX mặc định PA2 (AF7)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2 = Alternate Function
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));

    // AF7 cho PA2
    GPIOA->AFR[0] &= ~(0xFU << (2U * 4U));
    GPIOA->AFR[0] |=  (7U   << (2U * 4U));

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    uint32_t pclk1 = get_pclk1_hz();
    USART2->BRR = (uint16_t)(pclk1 / BAUDRATE); // oversampling 16

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
    for (int i = 0; i < len; i++)
    {
        if (ptr[i] == '\n') usart2_write_byte('\r');
        usart2_write_byte((uint8_t)ptr[i]);
    }
    return len;
}

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles--) { __NOP(); }
}

int main(void)
{
    usart2_init_tx_only();

    Led_CQ_Init(LED_CQ_ACTIVE_HIGH);
    Led_CQ_On();

    while (1)
    {
        printf("LED is ON (PC7)\n");
        delay_cycles(4000000U);
    }
}
