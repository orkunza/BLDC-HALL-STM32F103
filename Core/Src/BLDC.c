/**
  * @file       BLDC.c
  * @author     Orkun ZA (GitHub@orkunza)
  * @date       2021
  * @version    v1.0
  * @brief      This file is source file of BLDC.h
  * @details    BLDC driver
  */

#include "main.h"
#include "BLDC.h"
#include "stdbool.h"
#include "definitions.h"
#include "motorcontrolsettings.h"

#define BLDC_CHOPPER_PERIOD 4500

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c2;

extern uint8_t hallpos;

extern unsigned char phase;
extern unsigned char run;

void PWM_Timers_Init(void) {
	A_DOWN_PWM_START;
	B_DOWN_PWM_START;
	C_DOWN_PWM_START;
	A_UP_LOW;
	B_UP_LOW;
	C_UP_LOW;
	A_DOWN_PWM_DUTY = 0;
	B_DOWN_PWM_DUTY = 0;
	C_DOWN_PWM_DUTY = 0;
	A_DOWN_PWM_STOP;
	B_DOWN_PWM_STOP;
	C_DOWN_PWM_STOP;
}

void motorstartinit(void)
{
	//htim1.Instance->CCER = 0;
	//HAL_TIM_Base_Start(&htim1);
	A_DOWN_PWM_STOP;
	B_DOWN_PWM_STOP;
	C_DOWN_PWM_STOP;

	HallSensorsGetPosition();
	commutate();

	//__HAL_TIM_MOE_ENABLE(&htim1);
}



uint16_t adc_to_pwm(uint16_t adc_raw)
{
	uint16_t result = adc_raw * BLDC_CHOPPER_PERIOD / 4096;
	if (result > BLDC_CHOPPER_PERIOD)
	{
		result = BLDC_CHOPPER_PERIOD;
	}
	return result;
}

void HallSensorsGetPosition(void) {
	hallpos = (uint8_t) ((GPIOB->IDR & (H1_Pin | H2_Pin | H3_Pin)) >> 7); //0x0080 0x0100 0x0200
	switch (hallpos) {
	case 0b101:
		phase = 1;
		break;
	case 0b001:
		phase = 2;
		break;
	case 0b011:
		phase = 3;
		break;
	case 0b010:
		phase = 4;
		break;
	case 0b110:
		phase = 5;
		break;
	case 0b100:
		phase = 6;
		break;

	default:
		phase = 0;
		break;
	}
}

void commutate()
{
	//     60    120    180  240   300    360  Angle
	//     010   011    001  101   100    110  Hall_Sensor
	//     2     3      1    5     4      6
	//     c     b      a    f     d      e    Vektor

	switch (phase) {

	case 0:
		A_UP_LOW;
		B_UP_LOW;
		C_UP_LOW;
		A_DOWN_PWM_STOP;   	//        U V W   Hall
		B_DOWN_PWM_STOP;   	//0       0 0 0
		C_DOWN_PWM_STOP;   	//        0 0 0
		break;
	case 1:
		A_UP_HIGH;
		B_UP_LOW;
		C_UP_HIGH;
		A_DOWN_PWM_START;
		B_DOWN_PWM_STOP;
		C_DOWN_PWM_STOP;
		break;
	case 2:
		A_UP_LOW;
		B_UP_LOW;
		C_UP_HIGH;
		A_DOWN_PWM_START;
		B_DOWN_PWM_STOP;
		C_DOWN_PWM_STOP;
		break;
	case 3:
		A_UP_LOW;
		B_UP_LOW;
		C_UP_HIGH;
		A_DOWN_PWM_STOP;
		B_DOWN_PWM_START;
		C_DOWN_PWM_STOP;
		break;
	case 4:
		A_UP_HIGH;
		B_UP_LOW;
		C_UP_LOW;
		A_DOWN_PWM_STOP;
		B_DOWN_PWM_START;
		C_DOWN_PWM_STOP;
		break;
	case 5:
		A_UP_HIGH;
		B_UP_LOW;
		C_UP_LOW;
		A_DOWN_PWM_STOP;
		B_DOWN_PWM_STOP;
		C_DOWN_PWM_START;
		break;
	case 6:
		A_UP_LOW;
		B_UP_HIGH;
		C_UP_LOW;
		A_DOWN_PWM_STOP;
		B_DOWN_PWM_STOP;
		C_DOWN_PWM_START;
		break;
	} // end of phase switch statement
}




//int ADC_Ortala(int ADC_Ch, int Read_Times) {
//	ADC_Sum[ADC_Ch] = ADC_Sum[ADC_Ch] + ADC_Value[ADC_Ch];
//	Count[ADC_Ch]++;
//	if (Count[ADC_Ch] > Read_Times) {
//		ADC_Avr[ADC_Ch] = ADC_Sum[ADC_Ch] / Read_Times;
//		ADC_Sum[ADC_Ch] = 0;
//		Count[ADC_Ch] = 0;
//	}
//	return ADC_Avr[ADC_Ch];
//}
//float Ortala(float Val, float R_Times, int V_C) {
//	Val_Sum[V_C] = Val_Sum[V_C] + Val;
//	Val_Count[V_C]++;
//	if (Val_Count[V_C] > R_Times) {
//		Val_Avr[V_C] = Val_Sum[V_C] / R_Times;
//		Val_Sum[V_C] = 0;
//		Val_Count[V_C] = 0;
//	}
//	return Val_Avr[V_C];
//}
//
//float Limitations(float Value, float Upper_Limit, float Lower_Limit) {
//	if (Value >= Upper_Limit) {
//		Value = Upper_Limit;
//	}
//	if (Value <= Lower_Limit) {
//		Value = Lower_Limit;
//	}
//	return Value;
//}
//int Speed_Estimation(float coefficient) {
//	Counter2++;
//	float Signal_Speed;
//	if (Hall_Value == 1) {
//		__HAL_TIM_SET_COUNTER(&htim2, 0);
//	}
//	if (Hall_Value == 6) {
//		Signal_Duty = __HAL_TIM_GetCounter(&htim2);
//	}
//	Signal_Speed = 1000000 / Signal_Duty;
////Signal_Speed=Limitations(Signal_Speed,5000,0);
//	return coefficient * Signal_Speed;
//}
//
//
//
//
//void Motor_Start(float Ref_Start_Speed) {
//	if (Speed_PID_Lim > Ref_Start_Speed) {
//		//if(Speed>Ref_Start_Speed){
//		if (Motor_Run_Flag == 0) {
//			Motor_Run = 1;
//			Hall_Value = Hall_Sensor_Read();
//			BLDC_Six_Step_Driver_Alatay(Motor_Dir, Speed_PID_Lim);
//			Speed_Control_Duty();
//			Buzzer(10);
//		}
//	}
//	if (Motor_Speed == 0) {
//		Motor_Run = 0;
//		//BLDC_Six_Step_Driver_Alatay(Motor_Dir,0);
//		Vector_0();
//		Motor_Run_Flag = 0;
//	}
//}
//
//void PID_init(void) {
//	PID.Kp = PID_PARAM_KP; /* Proporcional */
//	PID.Ki = PID_PARAM_KI; /* Integral */
//	PID.Kd = PID_PARAM_KD; /* Derivative */
//
//	arm_pid_init_f32(&PID, 1);
//}
//
//int Current_Read(void) {
////Current_Total=ADC_Value[3];
//	Current_Total = ADC_Ortala(3, 500);
//	Current_Total = 2062 - Current_Total;
////Current_Total=Limitations(Current_Total,1000,0);
//	return Current_Total;
//}
//void Fault_Protocol() {
//	Speed_PID = 0;
//	Led_Red_On;
//	Buzzer(100);
//	Buzzer(100);
//	Buzzer(100);
//	Buzzer(100);
//	Buzzer(100);
//	Fault_Reset_High;
//	HAL_Delay(100);
//	Fault_Reset_Low;
//	HAL_Delay(100);
//	Led_Red_Off;
//}
//void Button_Control(void) {
//	if (Button1 == 0) {
//		Buzzer(100);
//		Motor_Run = 1;
//		Fault_Reset();
//	}
//	if (Button2 == 0) {
//		Motor_Run = 0;
//		if (Motor_Dir == 1) {
//			Motor_Dir = 0;
//		}
//		else if (Motor_Dir == 0) {
//			Motor_Dir = 1;
//		}
//		Buzzer(200);
//	}
//}
//void Referance_Speed_From_POT(void) {
//	Potantiometer_Digital = ADC_Ortala(0, 20);
//	Speed = Potantiometer_Digital / 2200.0 * 100.0;
//	Speed = Limitations(Speed, 100, 0);
//	Ref_Motor_Speed = 7 * Speed;
//	if (Ref_Motor_Speed < 10) {
//		Ref_Motor_Speed = 0;
//	}
//}
//void Speed_Control_Duty(void) {
//	Speed_Error = Ref_Motor_Speed - Motor_Speed_Filtered;
//
//	if (Speed_Error > 200) {
//		Speed_Error = 200;
//	}
//
//	if (Speed_PID_Lim < Motor_Start_Speed) {
//		Speed_PID = Speed_PID + Speed_Error * 0.006;
//	}
//	else {
//		Speed_PID = Speed_PID + Speed_Error * 0.0005;
//	}           //hata toparlama hizzi - hizlanma orani
//
////if(Speed_Error<0){Speed_PID=Speed_PID+Speed_Error*0.002;}
//
//	if (Ref_Motor_Speed == 0) {
//		Speed_PID = 0;
//	}
//	Speed_PID = Limitations(Speed_PID, 100, 0);
//	Speed_PID_Lim = Limitations(Speed_PID, 100, 0);
//}
//void Speed_Control_PID(void) {
//	pid_error = Ref_Motor_Speed - Motor_Speed;
//	if (pid_error < 0) {
//		pid_error = 4 * pid_error;
//	}
//	if (Speed_PID > 100) {
//		pid_error = -20;
//	}
//	Speed_PID = arm_pid_f32(&PID, pid_error);
//	if (Speed_PID < 0) {
//		arm_pid_reset_f32(&PID);
//	}
//	Speed_PID_Lim = Limitations(Speed_PID, 100, 0);
//}
//void Current_Limit_mA(float Limit_mAMPS) {
//	if (Current_T >= Limit_mAMPS) {
//		Current_Limit_Count++;
//		if (Current_Limit_Count > 500) {
//			Speed_PID -= 0.01f;
//			Current_Limit_Count = 500;
//			//Buzzer(1000);
//		}
//	}
//	if (Current_T >= Limit_mAMPS + 5000) {
//		Speed_PID = 0;
//		Current_Limit_Count = 500;
//	}
//}

