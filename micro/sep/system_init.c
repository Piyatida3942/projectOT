/*******************************************************************************
 * File Name    : system_init.c
 * Description  : Hardware initialization
 * Board        : NUCLEO-F411RE / Training Shield 1 Rev 02.00
 *******************************************************************************/

#include "system_init.h"
#include "led.h"

/*==============================================================================
 * Hardware Initialization
 *============================================================================*/
void System_Init(void)
{
    SysTick_Config(16000000u / 1000u);

    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
                     RCC_AHB1ENR_DMA2EN);
    RCC->APB1ENR |= (RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM4EN);
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN);

    /* LEDs Config */
    GPIOA->MODER &= ~((3u << (LED_STATUS_PIN * 2u)) |
                       (3u << (LED_RED_PIN * 2u)) |
                       (3u << (LED_YELLOW_PIN * 2u)));
    GPIOA->MODER |= ((1u << (LED_STATUS_PIN * 2u)) |
                     (1u << (LED_RED_PIN * 2u)) |
                     (1u << (LED_YELLOW_PIN * 2u)));

    GPIOB->MODER &= ~((3u << (LED_GREEN_PIN * 2u)) |
                       (3u << (LED_REJECT_PIN * 2u)) |
                       (3u << (LED_WAIT_LDR_PIN * 2u)));
    GPIOB->MODER |= ((1u << (LED_GREEN_PIN * 2u)) |
                     (1u << (LED_REJECT_PIN * 2u)) |
                     (1u << (LED_WAIT_LDR_PIN * 2u)));
    GPIOB->OTYPER &= ~((1u << LED_REJECT_PIN) | (1u << LED_WAIT_LDR_PIN));
    Clear_All_LEDs();

    /* Servo TIM4 PWM on PB8 */
    GPIOB->MODER &= ~(3u << (SERVO_PIN * 2u));
    GPIOB->MODER |=  (2u << (SERVO_PIN * 2u));
    GPIOB->AFR[1] &= ~(0xFu << ((SERVO_PIN - 8u) * 4u));
    GPIOB->AFR[1] |=  (2u << ((SERVO_PIN - 8u) * 4u));

    TIM4->PSC = 160u - 1u;
    TIM4->ARR = 2000u - 1u;
    TIM4->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    TIM4->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->CR1 |= TIM_CR1_CEN;
    TIM4->CCR3 = SERVO_POS_NORMAL;

    /* Analog inputs */
    GPIOA->MODER |= (3u << (LIGHT_SENSOR_PIN * 2u)) |
                    (3u << (POT_PIN * 2u));

    /* Buttons PB3, PB4, PB5 */
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 |
                      GPIO_MODER_MODER4 |
                      GPIO_MODER_MODER5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 |
                      GPIO_PUPDR_PUPD4 |
                      GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (1u << GPIO_PUPDR_PUPD3_Pos) |
                    (1u << GPIO_PUPDR_PUPD4_Pos) |
                    (1u << GPIO_PUPDR_PUPD5_Pos);

    /* Emergency PA10 */
    GPIOA->MODER &= ~(3u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->MODER |=  (2u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->PUPDR &= ~(3u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->PUPDR |=  (1u << (EMERGENCY_BUTTON_PIN * 2u));
    GPIOA->AFR[1] &= ~(0xFu << ((EMERGENCY_BUTTON_PIN - 8u) * 4u));
    GPIOA->AFR[1] |=  (1u << ((EMERGENCY_BUTTON_PIN - 8u) * 4u));

    TIM1->PSC = 16000u - 1u;
    TIM1->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM1->CCMR2 |= TIM_CCMR2_CC3S_0;
    TIM1->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
    TIM1->CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E;
    TIM1->DIER |= TIM_DIER_CC3IE;
    TIM1->SR &= ~TIM_SR_CC3IF;
    TIM1->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    /* IR PC10 */
    GPIOC->MODER &= ~(3u << (IR_PIN * 2u));
    GPIOC->PUPDR &= ~(3u << (IR_PIN * 2u));
    GPIOC->PUPDR |=  (1u << (IR_PIN * 2u));

    /* EXTI Interrupts Setup */
    SYSCFG->EXTICR[0] &= ~(0xFu << 12u);
    SYSCFG->EXTICR[0] |= (0x1u << 12u); /* EXTI3 PB3 */

    SYSCFG->EXTICR[1] &= ~((0xFu << 0u) | (0xFu << 4u));
    SYSCFG->EXTICR[1] |= ((0x1u << 0u) | (0x1u << 4u)); /* EXTI4 PB4, EXTI5 PB5 */

    SYSCFG->EXTICR[2] &= ~(0xFu << 8u);
    SYSCFG->EXTICR[2] |= (0x2u << 8u); /* EXTI10 PC10 */

    EXTI->IMR |= (EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR10);
    EXTI->FTSR |= (EXTI_FTSR_TR3 | EXTI_FTSR_TR4 | EXTI_FTSR_TR5 | EXTI_FTSR_TR10);
    EXTI->RTSR &= ~EXTI_RTSR_TR10;
    EXTI->PR |= (EXTI_PR_PR3 | EXTI_PR_PR4 | EXTI_PR_PR5 | EXTI_PR_PR10);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* UART2 Setup */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= ((2u << GPIO_MODER_MODER2_Pos) | (2u << GPIO_MODER_MODER3_Pos));
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= ((7u << GPIO_AFRL_AFSEL2_Pos) | (7u << GPIO_AFRL_AFSEL3_Pos));

    USART2->BRR = 0x8Bu;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE);
    NVIC_EnableIRQ(USART2_IRQn);

    /* ADC1 & DMA Setup */
    DMA2_Stream0->CR = 0u;
    while ((DMA2_Stream0->CR & DMA_SxCR_EN) != 0u) {
        /* รอจนกว่า stream จะปิดสนิทก่อนตั้งค่าใหม่ */
    }

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;
    DMA2_Stream0->NDTR = ADC_CHANNEL_COUNT;
    DMA2_Stream0->CR |= (0u << 25u) | (1u << 13u) | (1u << 11u) | (1u << 10u) | (1u << 8u);
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->SQR1 |= (1u << 20u);
    ADC1->SQR3 = (LIGHT_SENSOR_PIN << 0u) | (POT_PIN << 5u);
    ADC1->SMPR2 |= (7u << (LIGHT_SENSOR_PIN * 3u)) | (7u << (POT_PIN * 3u));
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}
