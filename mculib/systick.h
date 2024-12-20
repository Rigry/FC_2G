/*
 * systick.h
 *
 *  Created on: 9 февр. 2023 г.
 *      Author: Alex
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

class SystemTick {
public:
	template <uint16_t us> void initInterrupt()
	{
		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000 / (1000/us) - 1);
	}
}systemtick;



#endif /* SYSTICK_H_ */
