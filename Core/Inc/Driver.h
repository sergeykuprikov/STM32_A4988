#ifndef _Driver_H_
#define _Driver_H_


void Init_Driver(char step_resolution);

void Change_position(short position);

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_Driver_H_ */
