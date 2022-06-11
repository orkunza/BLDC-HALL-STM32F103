/**
  * @file       BLDC.h
  * @author     Orkun ZA (GitHub@orkunza)
  * @date       2021
  * @version    v1.1
  * @brief      This file is general header file of BLDC.h
  * @details    BLDC Driver
  */

#ifndef BLDC_H
#define BLDC_H

#include "main.h"
#include "math.h"
#include "definitions.h"
#include "motorcontrolsettings.h"


#  if __has_include("stm32f0xx_hal.h")
#    include "stm32f0xx_hal.h"
#  elif  __has_include("stm32f1xx_hal.h")
#    include "stm32f1xx_hal.h"
#  elif  __has_include("stm32f3xx_hal.h")
#    include "stm32f3xx_hal.h"
#  elif  __has_include("stm32f4xx_hal.h")
#    include "stm32f4xx_hal.h"
#  endif

#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @defgroup Select pins
 * @brief    74HC238 Select pins
 * @{
 */





void HallSensorsGetPosition(void);
void motorstartinit(void);
void commutate();
uint16_t adc_to_pwm(uint16_t adc_raw);

#define A_UP_HIGH   	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);	 // Set the Pin PB13
#define A_UP_LOW   		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);	  // Clear the Pin PB13

#define B_UP_HIGH   	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);			 // Set the Pin PB14
#define B_UP_LOW    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);  // Clear the Pin PB14

#define C_UP_HIGH  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);			 // Set the Pin PB15
#define C_UP_LOW    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);  // Clear the Pin PB15


//#define A_UP_HIGH   	GPIOB->BSRR |= (1 << 13);		 // Set the Pin PB13
//#define A_UP_LOW   		GPIOB->BSRR |= (1 << 13) << 16;  // Clear the Pin PB13
//
//#define B_UP_HIGH   	GPIOB->BSRR |= (1 << 14);		 // Set the Pin PB14
//#define B_UP_LOW    	GPIOB->BSRR |= (1 << 14) << 16;  // Clear the Pin PB14
//
//#define C_UP_HIGH  		GPIOB->BSRR |= (1 << 15);		 // Set the Pin PB15
//#define C_UP_LOW    	GPIOB->BSRR |= (1 << 15) << 16;  // Clear the Pin PB15

#define A_DOWN_PWM_DUTY 	htim1.Instance->CCR1
#define B_DOWN_PWM_DUTY 	htim1.Instance->CCR2
#define C_DOWN_PWM_DUTY  	htim1.Instance->CCR3
#define PWM_DUTY  			__HAL_TIM_SET_COMPARE

#define A_DOWN_PWM_START 	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
#define B_DOWN_PWM_START  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
#define C_DOWN_PWM_START  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

#define A_DOWN_PWM_STOP  	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
#define B_DOWN_PWM_STOP  	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
#define C_DOWN_PWM_STOP	 	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);


#define BUZZER_START  		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
#define BUZZER_STOP  		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
#define BUZZER_SET_DUTY   	htim2.Instance->CCR1






#define PID_PARAM_KP		0.00001		  	/* Proporcional */
#define PID_PARAM_KI		0.000001	  	/* Integral */
#define PID_PARAM_KD		0			        /* Derivative */

#define Motor_Start_Speed   1.5    // yüzde baslangic duty

	
void PWM_Timers_Init(void);
int Hall_Sensor_Read(void);
void BLDC_Six_Step_Driver_Alatay(int Motor_Direction,float Motor_Speed_Percent);
void BLDC_Six_Step_Driver_Alatay_Start(int Motor_Direction,float Motor_Speed_Percent);
int Speed_Estimation(float coefficient);
int Speed_Estimation_2(float coefficient);
void Fan_Control(void);
void Read_Temperature(void);
void Led_Intro(void);
void Fault_Reset(void);
void Motor_Start(float Ref_Start_Speed);
void Referance_Speed_From_POT(void);
void Button_Control(void);
float Ortala(float Val,float R_Times,int V_C);
void Fault_Protocol(void);

void PID_init(void);
void Speed_Control_PID(void);
void Speed_Control_Duty(void);

int ADC_Ortala(int ADC_Ch,int Read_Times);
float Limitations(float Value,float Upper_Limit, float Lower_Limit);
int Current_Read(void);
void Current_Limit_mA(float Limit_mAMPS);



#ifdef __cplusplus
}
#endif

#endif // end of BLDC.h
