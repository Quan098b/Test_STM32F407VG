#include "Thu_vien_L298.h"

/* ===== Helpers: GPIO clock enable ===== */
static void gpio_clock_enable(GPIO_TypeDef *g)
{
    if (g == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (g == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (g == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (g == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (g == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    else if (g == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    else if (g == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    (void)RCC->AHB1ENR;
}

static void gpio_output_pp(GPIO_TypeDef *g, uint8_t pin)
{
    g->MODER &= ~(3U << (pin * 2U));
    g->MODER |=  (1U << (pin * 2U)); /* output */
    g->OTYPER &= ~(1U << pin);       /* push-pull */
    g->PUPDR  &= ~(3U << (pin * 2U));/* no pull */
    g->OSPEEDR |= (1U << (pin * 2U));
}

static void gpio_set_af(GPIO_TypeDef *g, uint8_t pin, uint8_t af)
{
    g->MODER &= ~(3U << (pin * 2U));
    g->MODER |=  (2U << (pin * 2U)); /* AF */
    g->PUPDR  &= ~(3U << (pin * 2U));
    g->OSPEEDR |= (3U << (pin * 2U));

    uint32_t idx = pin >> 3;
    uint32_t sh  = (pin & 7U) * 4U;
    g->AFR[idx] &= ~(0xFU << sh);
    g->AFR[idx] |=  ((uint32_t)af << sh);
}

/* ===== Helpers: TIM clock enable ===== */
static void tim_clock_enable(TIM_TypeDef *t)
{
    if (t == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (t == TIM8) RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    else if (t == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (t == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (t == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (t == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    (void)RCC->APB1ENR; (void)RCC->APB2ENR;
}

static uint32_t get_pclk1_hz(void)
{
    SystemCoreClockUpdate();
    static const uint8_t apb_tbl[8] = {1,1,1,1,2,4,8,16};
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;
    return SystemCoreClock / apb_tbl[ppre1];
}

static uint32_t get_pclk2_hz(void)
{
    SystemCoreClockUpdate();
    static const uint8_t apb_tbl[8] = {1,1,1,1,2,4,8,16};
    uint32_t ppre2 = (RCC->CFGR >> 13) & 0x7U;
    return SystemCoreClock / apb_tbl[ppre2];
}

/* Timer clock: nếu APB prescaler != 1 thì TIMCLK = PCLK * 2 */
static uint32_t get_tim_clk_hz(TIM_TypeDef *t)
{
    uint32_t pclk, div;

    if (t == TIM1 || t == TIM8) {
        uint32_t ppre2 = (RCC->CFGR >> 13) & 0x7U;
        static const uint8_t apb_tbl[8] = {1,1,1,1,2,4,8,16};
        div  = apb_tbl[ppre2];
        pclk = get_pclk2_hz();
    } else {
        uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;
        static const uint8_t apb_tbl[8] = {1,1,1,1,2,4,8,16};
        div  = apb_tbl[ppre1];
        pclk = get_pclk1_hz();
    }

    return (div == 1U) ? pclk : (2U * pclk);
}

static void tim_pwm_config_channel(TIM_TypeDef *t, uint8_t ch)
{
    switch (ch) {
    case 1:
        t->CCMR1 &= ~(7U << 4);
        t->CCMR1 |=  (6U << 4);
        t->CCMR1 |=  (1U << 3);
        t->CCER  |=  (1U << 0);
        break;
    case 2:
        t->CCMR1 &= ~(7U << 12);
        t->CCMR1 |=  (6U << 12);
        t->CCMR1 |=  (1U << 11);
        t->CCER  |=  (1U << 4);
        break;
    case 3:
        t->CCMR2 &= ~(7U << 4);
        t->CCMR2 |=  (6U << 4);
        t->CCMR2 |=  (1U << 3);
        t->CCER  |=  (1U << 8);
        break;
    case 4:
        t->CCMR2 &= ~(7U << 12);
        t->CCMR2 |=  (6U << 12);
        t->CCMR2 |=  (1U << 11);
        t->CCER  |=  (1U << 12);
        break;
    default:
        break;
    }

    /* ARR preload */
    t->CR1 |= (1U << 7);
}

static void tim_pwm_set_ccr(TIM_TypeDef *t, uint8_t ch, uint32_t v)
{
    switch (ch) {
    case 1: t->CCR1 = v; break;
    case 2: t->CCR2 = v; break;
    case 3: t->CCR3 = v; break;
    case 4: t->CCR4 = v; break;
    default: break;
    }
}

/* ===== Internal: ENA GPIO on/off ===== */
static void ena_gpio_set(GPIO_TypeDef *g, uint8_t pin, uint8_t en)
{
    uint32_t b = 1U << pin;
    if (en) g->BSRR = b;
    else    g->BSRR = (b << 16);
}

/* ===================== PUBLIC API ===================== */

void L298_Init_GPIO(L298_Handle_t *h, const L298_Pins_t *pins)
{
    h->mode = L298_ENA_GPIO;
    h->pins = *pins;

    gpio_clock_enable(h->pins.ENA_Port);
    gpio_clock_enable(h->pins.IN1_Port);
    gpio_clock_enable(h->pins.IN2_Port);

    gpio_output_pp(h->pins.ENA_Port, h->pins.ENA_Pin);
    gpio_output_pp(h->pins.IN1_Port, h->pins.IN1_Pin);
    gpio_output_pp(h->pins.IN2_Port, h->pins.IN2_Pin);

    L298_SetDir(h, L298_DIR_STOP);
    L298_Enable(h, 0);
}

void L298_Init_PWM(L298_Handle_t *h, const L298_Pins_t *pins, const L298_PWM_t *pwm)
{
    h->mode = L298_ENA_PWM;
    h->pins = *pins;
    h->pwm  = *pwm;

    /* IN pins output */
    gpio_clock_enable(h->pins.IN1_Port);
    gpio_clock_enable(h->pins.IN2_Port);
    gpio_output_pp(h->pins.IN1_Port, h->pins.IN1_Pin);
    gpio_output_pp(h->pins.IN2_Port, h->pins.IN2_Pin);

    /* PWM pin AF */
    gpio_clock_enable(h->pwm.PWM_Port);
    gpio_set_af(h->pwm.PWM_Port, h->pwm.PWM_Pin, h->pwm.PWM_AF);

    /* Timer PWM init */
    tim_clock_enable(h->pwm.TIMx);

    uint32_t timclk = get_tim_clk_hz(h->pwm.TIMx);

    /* PSC để timer-count ~1MHz (gọn) */
    uint32_t target = 1000000U;
    uint32_t psc = (timclk / target);
    if (psc == 0) psc = 1;
    psc -= 1U;

    uint32_t cntclk = timclk / (psc + 1U);
    uint32_t arr = (cntclk / h->pwm.PWM_Hz);
    if (arr == 0) arr = 1;
    arr -= 1U;

    h->pwm.TIMx->CR1 = 0;
    h->pwm.TIMx->PSC = (uint16_t)psc;
    h->pwm.TIMx->ARR = (uint16_t)arr;

    tim_pwm_config_channel(h->pwm.TIMx, h->pwm.CH);

    /* Advanced timers cần MOE */
    if (h->pwm.TIMx == TIM1 || h->pwm.TIMx == TIM8) {
        h->pwm.TIMx->BDTR |= (1U << 15);
    }

    /* duty=0 */
    tim_pwm_set_ccr(h->pwm.TIMx, h->pwm.CH, 0);

    /* update event */
    h->pwm.TIMx->EGR |= 1U;
    h->pwm.TIMx->CR1 |= 1U; /* CEN */

    L298_SetDir(h, L298_DIR_STOP);
}

void L298_SetDir(L298_Handle_t *h, L298_Dir_t dir)
{
    uint32_t b1 = 1U << h->pins.IN1_Pin;
    uint32_t b2 = 1U << h->pins.IN2_Pin;

    if (dir == L298_DIR_FWD) {
        h->pins.IN1_Port->BSRR = b1;
        h->pins.IN2_Port->BSRR = (b2 << 16);
    } else if (dir == L298_DIR_REV) {
        h->pins.IN1_Port->BSRR = (b1 << 16);
        h->pins.IN2_Port->BSRR = b2;
    } else {
        /* STOP: IN1=0, IN2=0 */
        h->pins.IN1_Port->BSRR = (b1 << 16);
        h->pins.IN2_Port->BSRR = (b2 << 16);
    }
}

void L298_Enable(L298_Handle_t *h, uint8_t en)
{
    if (h->mode == L298_ENA_GPIO) {
        ena_gpio_set(h->pins.ENA_Port, h->pins.ENA_Pin, en);
    } else {
        /* ENA_PWM: enable = duty > 0, nhưng vẫn cho phép tắt nhanh bằng duty=0 */
        if (!en) {
            tim_pwm_set_ccr(h->pwm.TIMx, h->pwm.CH, 0);
        }
    }
}

void L298_SetDuty(L298_Handle_t *h, uint8_t duty_percent)
{
    if (h->mode != L298_ENA_PWM) return;

    if (duty_percent > 100U) duty_percent = 100U;

    uint32_t arr = h->pwm.TIMx->ARR;
    uint32_t ccr = ((uint32_t)duty_percent * (arr + 1U)) / 100U;
    if (ccr > arr) ccr = arr;

    tim_pwm_set_ccr(h->pwm.TIMx, h->pwm.CH, ccr);
}
