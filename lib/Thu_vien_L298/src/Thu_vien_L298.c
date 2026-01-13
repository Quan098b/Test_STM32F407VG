#include "Thu_vien_L298.h"

static void enable_gpio_clock(GPIO_TypeDef *gpio)
{
    if (gpio == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (gpio == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (gpio == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (gpio == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (gpio == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
}

static void gpio_output_pp(GPIO_TypeDef *gpio, uint8_t pin)
{
    gpio->MODER &= ~(3U << (pin * 2U));
    gpio->MODER |=  (1U << (pin * 2U));     /* output */
    gpio->OTYPER &= ~(1U << pin);           /* push-pull */
    gpio->PUPDR  &= ~(3U << (pin * 2U));    /* no pull */
    gpio->OSPEEDR |= (3U << (pin * 2U));    /* high speed */
}

static inline void pin_write(GPIO_TypeDef *gpio, uint8_t pin, uint8_t level)
{
    if (level) gpio->BSRR = (1U << pin);
    else       gpio->BSRR = (1U << (pin + 16U));
}

void L298_Init_SoftPWM(L298_Handle_t *h, const L298_Pins_t *pins, uint32_t pwm_period_ms)
{
    if (!h || !pins) return;

    h->pins = *pins;

    enable_gpio_clock(h->pins.ENA_Port);
    enable_gpio_clock(h->pins.IN1_Port);
    enable_gpio_clock(h->pins.IN2_Port);

    gpio_output_pp(h->pins.ENA_Port, h->pins.ENA_Pin);
    gpio_output_pp(h->pins.IN1_Port, h->pins.IN1_Pin);
    gpio_output_pp(h->pins.IN2_Port, h->pins.IN2_Pin);

    /* default safe state */
    pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, 0);
    pin_write(h->pins.IN1_Port, h->pins.IN1_Pin, 0);
    pin_write(h->pins.IN2_Port, h->pins.IN2_Pin, 0);

    h->pwm_period_ms  = (pwm_period_ms == 0U) ? 1U : pwm_period_ms;
    h->duty_percent   = 0;
    h->enabled        = 0;
    h->last_ena_level = 0;
}

void L298_SetDir(L298_Handle_t *h, L298_Dir_t dir)
{
    if (!h) return;

    if (dir == L298_DIR_FWD) {
        pin_write(h->pins.IN1_Port, h->pins.IN1_Pin, 1);
        pin_write(h->pins.IN2_Port, h->pins.IN2_Pin, 0);
    } else {
        pin_write(h->pins.IN1_Port, h->pins.IN1_Pin, 0);
        pin_write(h->pins.IN2_Port, h->pins.IN2_Pin, 1);
    }
}

void L298_Coast(L298_Handle_t *h)
{
    if (!h) return;
    pin_write(h->pins.IN1_Port, h->pins.IN1_Pin, 0);
    pin_write(h->pins.IN2_Port, h->pins.IN2_Pin, 0);
}

void L298_Enable(L298_Handle_t *h, uint8_t en)
{
    if (!h) return;
    h->enabled = (en ? 1U : 0U);

    if (!h->enabled) {
        pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, 0);
        h->last_ena_level = 0;
    }
}

void L298_SetDutyPercent(L298_Handle_t *h, uint8_t duty_percent)
{
    if (!h) return;
    if (duty_percent > 100U) duty_percent = 100U;
    h->duty_percent = duty_percent;
}

void L298_SoftPWM_Update(L298_Handle_t *h, uint32_t now_ms)
{
    if (!h) return;

    if (!h->enabled || h->duty_percent == 0U) {
        if (h->last_ena_level != 0U) {
            pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, 0);
            h->last_ena_level = 0;
        }
        return;
    }

    if (h->duty_percent >= 100U) {
        if (h->last_ena_level != 1U) {
            pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, 1);
            h->last_ena_level = 1;
        }
        return;
    }

    uint32_t phase = now_ms % h->pwm_period_ms;
    uint32_t on_ms = (h->pwm_period_ms * (uint32_t)h->duty_percent) / 100U;
    uint8_t ena_level = (phase < on_ms) ? 1U : 0U;

    if (ena_level != h->last_ena_level) {
        pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, ena_level);
        h->last_ena_level = ena_level;
    }
}
