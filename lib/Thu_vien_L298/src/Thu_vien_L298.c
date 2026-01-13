#include "Thu_vien_L298.h"

/* ---------- low-level helpers ---------- */
static inline void gpio_clk_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (port == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (port == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (port == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (port == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    else if (port == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    else if (port == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    else if (port == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
}

static inline void pin_mode_output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3U << (2U * (uint32_t)pin));
    port->MODER |=  (1U << (2U * (uint32_t)pin));     /* 01: output */
    port->OTYPER &= ~(1U << (uint32_t)pin);           /* push-pull */
    port->PUPDR  &= ~(3U << (2U * (uint32_t)pin));    /* no pull */
    port->OSPEEDR |= (3U << (2U * (uint32_t)pin));    /* high speed */
}

static inline void pin_write(GPIO_TypeDef *port, uint8_t pin, uint8_t val)
{
    if (val) port->BSRR = (1U << (uint32_t)pin);
    else     port->BSRR = (1U << ((uint32_t)pin + 16U));
}

/* ---------- API ---------- */
void L298_Init(L298_Handle_t *h, const L298_Pins_t *pins, uint32_t pwm_period_ms)
{
    if (!h || !pins) return;

    h->pins = *pins;

    /* clock enable */
    gpio_clk_enable(h->pins.ENA_Port);
    gpio_clk_enable(h->pins.IN1_Port);
    gpio_clk_enable(h->pins.IN2_Port);

    /* outputs */
    pin_mode_output(h->pins.ENA_Port, h->pins.ENA_Pin);
    pin_mode_output(h->pins.IN1_Port, h->pins.IN1_Pin);
    pin_mode_output(h->pins.IN2_Port, h->pins.IN2_Pin);

    /* default states */
    h->pwm_period_ms = (pwm_period_ms == 0U) ? 5U : pwm_period_ms;
    h->duty_percent  = 0U;
    h->enabled       = 0U;
    h->last_ena_level = 2U; /* invalid -> force update */

    /* coast + ENA low */
    pin_write(h->pins.IN1_Port, h->pins.IN1_Pin, 0);
    pin_write(h->pins.IN2_Port, h->pins.IN2_Pin, 0);
    pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, 0);
}

void L298_Enable(L298_Handle_t *h, uint8_t en)
{
    if (!h) return;
    h->enabled = en ? 1U : 0U;
    if (!h->enabled) {
        /* force ENA low immediately */
        pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, 0);
        h->last_ena_level = 0U;
    }
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

void L298_Brake(L298_Handle_t *h)
{
    if (!h) return;
    pin_write(h->pins.IN1_Port, h->pins.IN1_Pin, 1);
    pin_write(h->pins.IN2_Port, h->pins.IN2_Pin, 1);
}

void L298_SetDutyPercent(L298_Handle_t *h, uint8_t duty_percent)
{
    if (!h) return;
    if (duty_percent > 100U) duty_percent = 100U;
    h->duty_percent = duty_percent;
}

void L298_SetSpeed8(L298_Handle_t *h, uint8_t speed8)
{
    if (!h) return;
    /* map 0..255 -> 0..100 (%), rounding */
    uint32_t duty = ((uint32_t)speed8 * 100U + 127U) / 255U;
    if (duty > 100U) duty = 100U;
    h->duty_percent = (uint8_t)duty;
}

void L298_Task(L298_Handle_t *h, uint32_t now_ms)
{
    if (!h) return;

    if (!h->enabled || h->duty_percent == 0U) {
        if (h->last_ena_level != 0U) {
            pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, 0);
            h->last_ena_level = 0U;
        }
        return;
    }

    /* soft PWM */
    uint32_t period = (h->pwm_period_ms == 0U) ? 5U : h->pwm_period_ms;
    uint32_t phase  = now_ms % period;
    uint32_t on_ms  = (period * (uint32_t)h->duty_percent) / 100U;

    uint8_t ena = (phase < on_ms) ? 1U : 0U;

    if (ena != h->last_ena_level) {
        pin_write(h->pins.ENA_Port, h->pins.ENA_Pin, ena);
        h->last_ena_level = ena;
    }
}
