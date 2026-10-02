#include "stm32f4xx.h"

/* Simple delay */
void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms * 8000; i++)
    {
        __NOP();
    }
}

/* =========================================================
   TIM3 CH1 -> PA6
   Servo PWM:
   Timer clock = 16 MHz
   Timer tick  = 1 us
   PWM         = 50 Hz
   Period      = 20 ms
   ========================================================= */

void Servo_PWM_Init(void)
{
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= (1U << 0);

    /* Enable TIM3 clock */
    RCC->APB1ENR |= (1U << 1);

    /* -----------------------------------------------------
       PA6 -> Alternate Function mode
       ----------------------------------------------------- */
    GPIOA->MODER &= ~(3U << (6 * 2));
    GPIOA->MODER |=  (2U << (6 * 2));

    /* PA6 -> AF2 = TIM3_CH1 */
    GPIOA->AFR[0] &= ~(0xFU << (6 * 4));
    GPIOA->AFR[0] |=  (2U << (6 * 4));

    /* Push-pull */
    GPIOA->OTYPER &= ~(1U << 6);

    /* -----------------------------------------------------
       TIM3

       16 MHz / 16 = 1 MHz
       Therefore 1 count = 1 us

       20,000 us period = 50 Hz
       ----------------------------------------------------- */
    TIM3->PSC = 15;
    TIM3->ARR = 19999;

    /* PWM Mode 1 on Channel 1 */
    TIM3->CCMR1 &= ~(7U << 4);
    TIM3->CCMR1 |=  (6U << 4);

    /* Enable preload for CCR1 */
    TIM3->CCMR1 |= (1U << 3);

    /* Enable TIM3 Channel 1 */
    TIM3->CCER |= (1U << 0);

    /* Start at center */
    TIM3->CCR1 = 1500;

    /* Auto-reload preload */
    TIM3->CR1 |= (1U << 7);

    /* Generate update event */
    TIM3->EGR |= (1U << 0);

    /* Start TIM3 */
    TIM3->CR1 |= (1U << 0);
}


/* =========================================================
   Set servo angle

   0    -> 1000 us
   90   -> 1500 us
   180  -> 2000 us
   ========================================================= */

void Servo_SetAngle(uint8_t angle)
{
    uint32_t pulse;

    if(angle > 180)
        angle = 180;

    pulse = 1000 + ((uint32_t)angle * 1000) / 180;

    TIM3->CCR1 = pulse;
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    Servo_PWM_Init();

    while(1)
    {
        /* Center */
        Servo_SetAngle(90);
        delay_ms(1000);

        /* One side */
        Servo_SetAngle(45);
        delay_ms(1000);

        /* Center */
        Servo_SetAngle(90);
        delay_ms(1000);

        /* Other side */
        Servo_SetAngle(135);
        delay_ms(1000);
    }
}
