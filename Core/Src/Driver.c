#include "stm32f1xx_hal.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

#define PWM &htim2
/* Выбор шага */

//	short step_resolution = 2;

	int _counter = 0;
	short _next_position = 0;
	short _current_position = 1;
	short _ticks_to_next = 0;
	short _options = 0;


void Init_Driver(short step_resolution)
{
	switch (step_resolution)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOB, MS1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MS2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MS3_Pin, GPIO_PIN_RESET);
			_ticks_to_next = 13;
			HAL_Delay(1000);
			break;

		case 2:
			HAL_GPIO_WritePin(GPIOB, MS1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MS2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MS3_Pin, GPIO_PIN_RESET);
			_ticks_to_next = 50;
			HAL_Delay(1000);
			break;

		case 3:
			HAL_GPIO_WritePin(GPIOB, MS1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MS2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MS3_Pin, GPIO_PIN_RESET);
			_ticks_to_next = 100;
			HAL_Delay(1000);
			break;

		case 4:
			HAL_GPIO_WritePin(GPIOB, MS1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MS2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MS3_Pin, GPIO_PIN_RESET);
			_ticks_to_next = 200;
			HAL_Delay(1000);
			break;

		case 5:
			HAL_GPIO_WritePin(GPIOB, MS1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MS2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MS3_Pin, GPIO_PIN_SET);
			_ticks_to_next = 400;
			HAL_Delay(1000);
			break;

		default:
			HAL_GPIO_WritePin(GPIOB, MS2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MS2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MS3_Pin, GPIO_PIN_RESET);
			_ticks_to_next = 50;
			HAL_Delay(1000);
			break;

	}
	HAL_GPIO_WritePin(GPIOB, ENABLE_Pin, GPIO_PIN_SET);
}

void Change_position(short position)
{
	_counter = 0;
	_next_position = position;
	HAL_GPIO_WritePin(GPIOB, ENABLE_Pin, GPIO_PIN_RESET);
	if (_next_position != _current_position)
	{
		if (_next_position > _current_position)
		{
			if ((_next_position - _current_position) <= 8)
			{
				HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_RESET);
				_options = (_next_position - _current_position);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_SET);
				_options = (16 - (_next_position - _current_position));
			}
		}
		if (_next_position < _current_position)
		{
			if ((_current_position - _next_position) < 8)
			{
				HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_SET);
				_options = (_current_position - _next_position);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_RESET);
				_options = (16 - (_current_position - _next_position));
			}
		}

		HAL_TIM_PWM_Start_IT(PWM, TIM_CHANNEL_1);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == PWM)
	{
		_counter += 1;
		if (_counter == (_options * _ticks_to_next))
		{
			HAL_TIM_PWM_Stop_IT(PWM, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOB, ENABLE_Pin, GPIO_PIN_SET);
			_current_position = _next_position;
		}
	}
}
