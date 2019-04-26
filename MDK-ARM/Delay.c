#include "Delay.h"
void delay(uint16_t delay_time, TIM_HandleTypeDef *timer_handle)
{
		timer_handle->Instance->CNT = 0;
		while(__HAL_TIM_GetCounter(timer_handle) <= delay_time);
}
