#include "Motor_Encoder_Cua_Quan.h"

/* LUT quadrature chuẩn: state=(B<<1)|A; idx=(prev<<2)|curr */
static const int8_t ENC_LUT[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t read_pin(GPIO_TypeDef *gpio, uint8_t pin)
{
    return (gpio->IDR & (1U << pin)) ? 1U : 0U;
}

static inline uint8_t read_ab(EncoderEXTI_CQ_Handle *h)
{
    uint8_t a = read_pin(h->gpioA, h->pinA);
    uint8_t b = read_pin(h->gpioB, h->pinB);
    return (uint8_t)((b << 1) | a);
}

static void gpio_input_pullup(GPIO_TypeDef *gpio, uint8_t pin)
{
    gpio->MODER &= ~(3U << (pin * 2U));      /* input */
    gpio->PUPDR &= ~(3U << (pin * 2U));
    gpio->PUPDR |=  (1U << (pin * 2U));      /* pull-up */
}

/* Map EXTI line to port:
   EXTICR index = line/4, shift = (line%4)*4
   portcode: A=0, B=1, C=2, D=3, E=4 ...
*/
static void exti_map_line_to_port(uint8_t line, uint8_t portcode)
{
    uint32_t idx = line / 4U;
    uint32_t sh  = (line % 4U) * 4U;

    SYSCFG->EXTICR[idx] &= ~(0xFU << sh);
    SYSCFG->EXTICR[idx] |=  ((uint32_t)portcode << sh);
}

void EncoderEXTI_CQ_Init(EncoderEXTI_CQ_Handle *h)
{
    if (!h || !h->gpioA || !h->gpioB) return;

    /* Enable GPIO clocks (AHB1) */
    if (h->gpioA == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if (h->gpioA == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if (h->gpioA == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if (h->gpioA == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if (h->gpioA == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    if (h->gpioB == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if (h->gpioB == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if (h->gpioB == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if (h->gpioB == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if (h->gpioB == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    /* SYSCFG for EXTI mapping */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    gpio_input_pullup(h->gpioA, h->pinA);
    gpio_input_pullup(h->gpioB, h->pinB);

    /* Determine portcode from gpio pointer */
    uint8_t portA = 0, portB = 0;
    if (h->gpioA == GPIOA) portA = 0;
    else if (h->gpioA == GPIOB) portA = 1;
    else if (h->gpioA == GPIOC) portA = 2;
    else if (h->gpioA == GPIOD) portA = 3;
    else if (h->gpioA == GPIOE) portA = 4;

    if (h->gpioB == GPIOA) portB = 0;
    else if (h->gpioB == GPIOB) portB = 1;
    else if (h->gpioB == GPIOC) portB = 2;
    else if (h->gpioB == GPIOD) portB = 3;
    else if (h->gpioB == GPIOE) portB = 4;

    /* Map lines */
    exti_map_line_to_port(h->pinA, portA);
    exti_map_line_to_port(h->pinB, portB);

    /* Enable EXTI both edges */
    EXTI->IMR  |= (1U << h->pinA) | (1U << h->pinB);
    EXTI->RTSR |= (1U << h->pinA) | (1U << h->pinB);
    EXTI->FTSR |= (1U << h->pinA) | (1U << h->pinB);

    /* Clear pending */
    EXTI->PR = (1U << h->pinA) | (1U << h->pinB);

    h->count = 0;
    h->prev_ab = read_ab(h);

    /* NOTE: PC7/PC9 nằm trong EXTI9_5_IRQn */
    NVIC_SetPriority(EXTI9_5_IRQn, 5);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EncoderEXTI_CQ_IRQHandler(EncoderEXTI_CQ_Handle *h)
{
    if (!h) return;

    uint32_t mask = (1U << h->pinA) | (1U << h->pinB);
    uint32_t pr = EXTI->PR;

    if (pr & mask)
    {
        /* clear pending */
        EXTI->PR = mask;

        uint8_t curr = read_ab(h);
        uint8_t idx  = (uint8_t)((h->prev_ab << 2) | curr);
        int8_t d = ENC_LUT[idx];

        if (d) h->count += d;
        h->prev_ab = curr;
    }
}

int32_t EncoderEXTI_CQ_GetCount(EncoderEXTI_CQ_Handle *h)
{
    if (!h) return 0;
    __disable_irq();
    int32_t c = h->count;
    __enable_irq();
    return c;
}

void EncoderEXTI_CQ_Reset(EncoderEXTI_CQ_Handle *h)
{
    if (!h) return;
    __disable_irq();
    h->count = 0;
    __enable_irq();
}
