#include "stm32f4xx.h"
#include <stdio.h>

#define BAUDRATE 115200U

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles--) { __NOP(); }
}

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

    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));
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

/* ===== L298 GPIO: ENA=PC8, IN1=PC13, IN2=PE5 ===== */
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

static void l298_forward(void)
{
    /* IN1=1, IN2=0 */
    GPIOC->BSRR = (1U << 13U);
    GPIOE->BSRR = (1U << (5U + 16U));
}

static void l298_enable(uint8_t en)
{
    if (en) GPIOC->BSRR = (1U << 8U);          /* ENA=1 */
    else    GPIOC->BSRR = (1U << (8U + 16U));  /* ENA=0 */
}

int main(void)
{
    usart2_init_tx_only();
    l298_gpio_init();

    printf("START: L298 test (ENA=PC8, IN1=PC13, IN2=PE5)\n");

    l298_forward();
    l298_enable(1);

    while (1)
    {
        printf("Motor should be running (ENA=1)\n");
        delay_cycles(16000000U);

        l298_enable(0);
        printf("Motor OFF (ENA=0)\n");
        delay_cycles(16000000U);

        l298_enable(1);
    }
}
