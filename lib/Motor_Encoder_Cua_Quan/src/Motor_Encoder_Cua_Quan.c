#include "Motor_Encoder_Cua_Quan.h"

/* ====== GPIO clock enable helper ====== */
static void gpio_clock_enable(GPIO_TypeDef *g)
{
    if (g == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (g == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (g == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (g == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (g == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;
}

static void gpio_input(GPIO_TypeDef *g, uint8_t pin)
{
    g->MODER &= ~(3U << (pin * 2U)); /* input */
    g->PUPDR &= ~(3U << (pin * 2U)); /* nopull (tùy encoder có thể cần pull-up) */
}

static void gpio_output_pp(GPIO_TypeDef *g, uint8_t pin)
{
    g->MODER &= ~(3U << (pin * 2U));
    g->MODER |=  (1U << (pin * 2U)); /* output */
    g->OTYPER &= ~(1U << pin);       /* push-pull */
    g->PUPDR  &= ~(3U << (pin * 2U));/* nopull */
}

/* ====== EXTI mapping helper (pin -> EXTICR index/shift) ====== */
static void exti_map_to_port(uint8_t line, uint8_t portcode)
{
    /* EXTICR[0] lines 0..3, [1] 4..7, [2] 8..11, [3] 12..15 */
    uint8_t idx = line / 4U;
    uint8_t sh  = (line % 4U) * 4U;
    SYSCFG->EXTICR[idx] &= ~(0xFU << sh);
    SYSCFG->EXTICR[idx] |=  ((uint32_t)portcode << sh);
}

static uint8_t port_to_code(GPIO_TypeDef *g)
{
    if (g == GPIOA) return 0;
    if (g == GPIOB) return 1;
    if (g == GPIOC) return 2;
    if (g == GPIOD) return 3;
    if (g == GPIOE) return 4;
    return 0;
}

/* ====== Quadrature lookup: prev(2-bit) -> now(2-bit) ======
   Table gives delta in {-1,0,+1}. */
static const int8_t quad_table[4][4] = {
    /* now: 00, 01, 10, 11 */
    /* prev 00 */ {  0, +1, -1,  0 },
    /* prev 01 */ { -1,  0,  0, +1 },
    /* prev 10 */ { +1,  0,  0, -1 },
    /* prev 11 */ {  0, -1, +1,  0 }
};

static uint8_t read_ab(GPIO_TypeDef *ga, uint8_t pa, GPIO_TypeDef *gb, uint8_t pb)
{
    uint8_t a = (uint8_t)((ga->IDR >> pa) & 1U);
    uint8_t b = (uint8_t)((gb->IDR >> pb) & 1U);
    return (uint8_t)((a << 1) | b);
}

/* ===================== ENCODER EXTI ===================== */
void EncoderEXTI_CQ_Init(EncoderEXTI_CQ_Handle *h)
{
    gpio_clock_enable(h->gpioA);
    gpio_clock_enable(h->gpioB);

    gpio_input(h->gpioA, h->pinA);
    gpio_input(h->gpioB, h->pinB);

    /* Enable SYSCFG for EXTI routing */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    uint8_t lineA = h->pinA;
    uint8_t lineB = h->pinB;

    exti_map_to_port(lineA, port_to_code(h->gpioA));
    exti_map_to_port(lineB, port_to_code(h->gpioB));

    /* Unmask lines */
    EXTI->IMR |= (1U << lineA) | (1U << lineB);

    /* Trigger both edges for quadrature */
    EXTI->RTSR |= (1U << lineA) | (1U << lineB);
    EXTI->FTSR |= (1U << lineA) | (1U << lineB);

    /* Clear pending */
    EXTI->PR = (1U << lineA) | (1U << lineB);

    /* PC7 + PC9 đều thuộc nhóm EXTI9_5 */
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    h->count = 0;
    h->last_ab = read_ab(h->gpioA, h->pinA, h->gpioB, h->pinB);
}

void EncoderEXTI_CQ_IRQHandler(EncoderEXTI_CQ_Handle *h)
{
    uint32_t maskA = (1U << h->pinA);
    uint32_t maskB = (1U << h->pinB);

    uint32_t pr = EXTI->PR;

    if (pr & (maskA | maskB)) {
        /* Clear pending first */
        EXTI->PR = (maskA | maskB);

        uint8_t now = read_ab(h->gpioA, h->pinA, h->gpioB, h->pinB);
        int8_t d = quad_table[h->last_ab][now];
        h->count += d;
        h->last_ab = now;
    }
}

int16_t EncoderEXTI_CQ_GetDelta(EncoderEXTI_CQ_Handle *h)
{
    /* Delta = count since last call => cần biến lưu last_count cục bộ tĩnh theo handle.
       Cách gọn: dùng static 1 handle/1 encoder. Nếu bạn dùng >1 encoder, mở rộng bằng field last_count. */
    static int32_t last = 0;
    int32_t now = h->count;
    int16_t d = (int16_t)(now - last);
    last = now;
    return d;
}

int32_t EncoderEXTI_CQ_GetCount(EncoderEXTI_CQ_Handle *h)
{
    return h->count;
}

void EncoderEXTI_CQ_Reset(EncoderEXTI_CQ_Handle *h)
{
    h->count = 0;
    h->last_ab = read_ab(h->gpioA, h->pinA, h->gpioB, h->pinB);
}

/* ===================== MOTOR L298 GPIO ===================== */
void MotorL298_CQ_Init(MotorL298_CQ_Handle *m)
{
    gpio_clock_enable(m->gpio_ena);
    gpio_clock_enable(m->gpio_in1);
    gpio_clock_enable(m->gpio_in2);

    gpio_output_pp(m->gpio_ena, m->pin_ena);
    gpio_output_pp(m->gpio_in1, m->pin_in1);
    gpio_output_pp(m->gpio_in2, m->pin_in2);

    MotorL298_CQ_Enable(m, 0);
    MotorL298_CQ_SetDir(m, 0);
}

void MotorL298_CQ_Enable(MotorL298_CQ_Handle *m, uint8_t en)
{
    uint32_t b = (1U << m->pin_ena);
    if (en) m->gpio_ena->BSRR = b;
    else    m->gpio_ena->BSRR = (b << 16);
}

void MotorL298_CQ_SetDir(MotorL298_CQ_Handle *m, int8_t dir)
{
    uint32_t b1 = (1U << m->pin_in1);
    uint32_t b2 = (1U << m->pin_in2);

    if (dir > 0) {
        /* forward: IN1=1, IN2=0 */
        m->gpio_in1->BSRR = b1;
        m->gpio_in2->BSRR = (b2 << 16);
    } else if (dir < 0) {
        /* reverse: IN1=0, IN2=1 */
        m->gpio_in1->BSRR = (b1 << 16);
        m->gpio_in2->BSRR = b2;
    } else {
        /* stop: IN1=0, IN2=0 */
        m->gpio_in1->BSRR = (b1 << 16);
        m->gpio_in2->BSRR = (b2 << 16);
    }
}
