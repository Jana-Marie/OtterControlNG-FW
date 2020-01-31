#define PWM_U TIM1->CCR3
#define PWM_V TIM1->CCR2
#define PWM_W TIM1->CCR1
#define PWM_TIM TIM1

#define PWM_DEADTIME 40 // 40/170e6 = 235ns
#define PWM_RES 4250 // 170e6 / 4250 = 40kHz center aligned pwm

#define HALL_A_PIN GPIO_PIN_6
#define HALL_A_PORT GPIOB
#define HALL_B_PIN GPIO_PIN_7
#define HALL_B_PORT GPIOB
#define HALL_C_PIN GPIO_PIN_8
#define HALL_C_PORT GPIOB

#define UART USART2
#define UART_SPEED 9600