#include "Led_Cua_Quan.h"

static Led_CQ_ActiveLevel g_level = LED_CQ_ACTIVE_HIGH;

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles--) { __NOP(); }
}

void Led_CQ_Init(Led_CQ_ActiveLevel level)
{
    g_level = level;

    /* Enable clock cho GPIOC */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /* PC7 output */
    GPIOC->MODER &= ~(3U << (7U * 2U));
    GPIOC->MODER |=  (1U << (7U * 2U));

    /* Push-pull */
    GPIOC->OTYPER &= ~(1U << 7U);

    /* No pull-up/down */
    GPIOC->PUPDR &= ~(3U << (7U * 2U));

    /* Medium speed (tuỳ chọn) */
    GPIOC->OSPEEDR &= ~(3U << (7U * 2U));
    GPIOC->OSPEEDR |=  (1U << (7U * 2U));

    /* Tắt LED ban đầu */
    Led_CQ_Off();
}

void Led_CQ_On(void)
{
    if (g_level == LED_CQ_ACTIVE_HIGH) {
        GPIOC->BSRR = (1U << 7U);          /* SET PC7 */
    } else {
        GPIOC->BSRR = (1U << (7U + 16U));  /* RESET PC7 */
    }
}

void Led_CQ_Off(void)
{
    if (g_level == LED_CQ_ACTIVE_HIGH) {
        GPIOC->BSRR = (1U << (7U + 16U));  /* RESET PC7 */
    } else {
        GPIOC->BSRR = (1U << 7U);          /* SET PC7 */
    }
}

void Led_CQ_Toggle(void)
{
    GPIOC->ODR ^= (1U << 7U);
}

void Led_CQ_BlinkDelay(volatile uint32_t cycles)
{
    Led_CQ_Toggle();
    delay_cycles(cycles);
}
