#include "Motor_Encoder_Cua_Quan.h"

/* PC6=TIM3_CH1 (AF2), PC7=TIM3_CH2 (AF2) */
static inline void gpio_pc6_pc7_af2_input_pullup(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* Alternate function mode */
    GPIOC->MODER &= ~(3U << (2U * 6U));
    GPIOC->MODER &= ~(3U << (2U * 7U));
    GPIOC->MODER |=  (2U << (2U * 6U));
    GPIOC->MODER |=  (2U << (2U * 7U));

    /* Pull-up (an toàn cho encoder open-collector; nếu encoder push-pull cũng không sao) */
    GPIOC->PUPDR &= ~(3U << (2U * 6U));
    GPIOC->PUPDR &= ~(3U << (2U * 7U));
    GPIOC->PUPDR |=  (1U << (2U * 6U));
    GPIOC->PUPDR |=  (1U << (2U * 7U));

    /* AF2 */
    GPIOC->AFR[0] &= ~(0xFU << (4U * 6U));
    GPIOC->AFR[0] &= ~(0xFU << (4U * 7U));
    GPIOC->AFR[0] |=  (2U   << (4U * 6U));
    GPIOC->AFR[0] |=  (2U   << (4U * 7U));
}

void EncoderEXTI_CQ_Init(EncoderEXTI_CQ_Handle *h)
{
    if (!h) return;

    h->tim = TIM3;
    h->count32 = 0;
    h->prev_cnt = 0;

    gpio_pc6_pc7_af2_input_pullup();

    /* TIM3 clock enable */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Stop counter */
    TIM3->CR1 &= ~TIM_CR1_CEN;

    /* Base config */
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;

    /* Encoder mode 3 (x4): SMS=011 */
    TIM3->SMCR &= ~TIM_SMCR_SMS;
    TIM3->SMCR |=  (3U << TIM_SMCR_SMS_Pos);

    /* CC1S=01 (TI1), CC2S=01 (TI2) */
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM3->CCMR1 |=  (1U << TIM_CCMR1_CC1S_Pos);
    TIM3->CCMR1 |=  (1U << TIM_CCMR1_CC2S_Pos);

    /* Digital filter chống nhiễu mức vừa: IC1F=3, IC2F=3 */
    TIM3->CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);
    TIM3->CCMR1 |=  (3U << TIM_CCMR1_IC1F_Pos);
    TIM3->CCMR1 |=  (3U << TIM_CCMR1_IC2F_Pos);

    /* Polarity mặc định (rising), enable capture */
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM3->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

    /* Init prev */
    h->prev_cnt = (uint16_t)TIM3->CNT;

    /* Start */
    TIM3->CR1 |= TIM_CR1_CEN;
}

void EncoderEXTI_CQ_Task(EncoderEXTI_CQ_Handle *h)
{
    if (!h || !h->tim) return;

    uint16_t curr = (uint16_t)h->tim->CNT;
    int16_t d = (int16_t)(curr - h->prev_cnt); /* tự xử lý wrap 16-bit */

    h->count32 += (int32_t)d;
    h->prev_cnt = curr;
}

int32_t EncoderEXTI_CQ_GetCount(EncoderEXTI_CQ_Handle *h)
{
    if (!h) return 0;
    __disable_irq();
    int32_t c = h->count32;
    __enable_irq();
    return c;
}

void EncoderEXTI_CQ_Reset(EncoderEXTI_CQ_Handle *h)
{
    if (!h || !h->tim) return;
    __disable_irq();
    h->tim->CNT = 0;
    h->prev_cnt = 0;
    h->count32  = 0;
    __enable_irq();
}
