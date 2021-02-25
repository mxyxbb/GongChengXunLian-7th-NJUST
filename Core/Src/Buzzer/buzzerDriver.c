/*
 * buzzerDriver.c
 *
 *  Created on: Feb 22, 2019
 *      Author: lukasz
 */
#include "main.h"
#include "buzzerDriver.h"
#include "melody.h"

#define CPU_FREQ	168000000
#define PRESCALER 	168

/*
 * Buzzer driver init
 */
inline void buzzerDriverInit(void)
{
	//Base Timer configuration:
	TIM9 -> CR1 &= ~(TIM_CR1_CKD_0 | TIM_CR1_CKD_1 | TIM_CR1_DIR);	// TIME x1 and DIR UP
	TIM9 -> CR1 |= TIM_CR1_ARPE;									// Auto reload
	TIM9 -> PSC = PRESCALER - 1;									// Prescaler
	TIM2 ->ARR = 60000;												// Period

	//Pulse
	TIM9 -> CCER &= ~TIM_CCER_CC4E;
	TIM9 -> CCER |= TIM_CCER_CC4P;

	TIM9 -> CCMR2 &= ~TIM_CR2_OIS4;
	TIM9 -> CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

	//Set pulse
	TIM9 -> CCR1 = 60000/2;
}

/*
 * 	Set new frequency
 *
 * 	args:
 * 	newFreq - new frequency in Hz.
 */
void buzzerSetNewFrequency(uint32_t newFreq)
{
	uint64_t tempFreq = newFreq;
	if(newFreq != 0){

	uint64_t tempNewValue = (uint64_t) CPU_FREQ / PRESCALER / tempFreq;

	// setting new value
	TIM9 ->ARR = (uint32_t)tempNewValue;
	TIM9 -> CCR1 = (uint32_t)tempNewValue/2;
	}
	else{
		TIM9 -> CCR1 = 0;
	}
}

void musicPlay()
{
	buzzerDriverInit();

  TIM9 -> CR1 |= TIM_CR1_CEN;
  TIM9 ->CCER |= TIM_CCER_CC4E;

  int melodyCount = sizeof(melodySizes)/ sizeof(uint32_t);

  for(int melodyIndex = 0; melodyIndex < melodyCount; melodyIndex++)
  {
	  for(int noteIndex = 0; noteIndex < melodySizes[melodyIndex]; noteIndex++)
  	  {
	  	  buzzerSetNewFrequency(melody[melodyIndex][noteIndex]);
	  	  HAL_Delay(noteDurations[melodyIndex][noteIndex] * melodySlowfactor[melodyIndex]);
  	  }
  }
   TIM9 -> CR1 &= ~TIM_CR1_CEN;
   TIM9 ->CCER &= ~TIM_CCER_CC4E;
}
