/*
 * buzzerDriver.c
 *
 *  Created on: Feb 22, 2019
 *      Author: lukasz
 */
#include "main.h"
#include "buzzerDriver.h"
#include "melody.h"
#include "stdbool.h"
#include "mymath.h"

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
	TIM9 ->ARR = 60000;												// Period

	//Pulse
	TIM9 -> CCER &= ~TIM_CCER_CC4E;
	TIM9 -> CCER |= TIM_CCER_CC4P;

	TIM9 -> CCMR2 &= ~TIM_CR2_OIS4;
	TIM9 -> CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

	//Set pulse
	TIM9 -> CCR1 = 60000/2;
	
	//Base Timer configuration:
	TIM12 -> CR1 &= ~(TIM_CR1_CKD_0 | TIM_CR1_CKD_1 | TIM_CR1_DIR);	// TIME x1 and DIR UP
	TIM12 -> CR1 |= TIM_CR1_ARPE;									// Auto reload
	TIM12 -> PSC = PRESCALER - 1;									// Prescaler
	TIM12 ->ARR = 60000;												// Period

	//Pulse
	TIM12 -> CCER &= ~TIM_CCER_CC4E;
	TIM12 -> CCER |= TIM_CCER_CC4P;

	TIM12 -> CCMR2 &= ~TIM_CR2_OIS4;
	TIM12 -> CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

	//Set pulse
	TIM12 -> CCR2 = 60000/2;
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
void buzzerSetNewFrequency2(uint32_t newFreq)
{
	uint64_t tempFreq = newFreq;
	if(newFreq != 0){

	uint64_t tempNewValue = (uint64_t) CPU_FREQ / PRESCALER / tempFreq;

	// setting new value
	TIM12 ->ARR = (uint32_t)tempNewValue;
	TIM12 -> CCR2 = (uint32_t)tempNewValue/2;
	}
	else{
		TIM12 -> CCR2 = 0;
	}
}


//void musicPlay()
//{
//	__ExecuteOnce(buzzerDriverInit());

//  TIM9 -> CR1 |= TIM_CR1_CEN;
//  TIM9 ->CCER |= TIM_CCER_CC4E;

//  int melodyCount = sizeof(melodySizes)/ sizeof(uint32_t);

//  for(int melodyIndex = 0; melodyIndex < melodyCount; melodyIndex++)
//  {
//	  for(int noteIndex = 0; noteIndex < melodySizes[melodyIndex]; noteIndex++)
//  	  {
//	  	  buzzerSetNewFrequency(melody[melodyIndex][noteIndex]);
//	  	  HAL_Delay(noteDurations[melodyIndex][noteIndex] * melodySlowfactor[melodyIndex]);
//  	  }
//  }
//   TIM9 -> CR1 &= ~TIM_CR1_CEN;
//   TIM9 ->CCER &= ~TIM_CCER_CC4E;
//}
//·Ç¶ÂÈû
void musicPlay()
{
	__ExecuteOnce(buzzerDriverInit());
	__ExecuteOnce(TIM9 -> CR1 |= TIM_CR1_CEN);
	__ExecuteOnce(TIM9 ->CCER |= TIM_CCER_CC4E);
//  TIM9 -> CR1 |= TIM_CR1_CEN;
//  TIM9 ->CCER |= TIM_CCER_CC4E;
	static uint32_t cnt_=0;
//  static int melodyCount = sizeof(melodySizes)/ sizeof(uint32_t);
//	static int melodyIndex = 0;
	static int noteIndex = 0;
	if( noteIndex >= melodySizes[0]) noteIndex=0;
	if(cnt_++%(noteDurations[0][noteIndex] * melodySlowfactor[0])==0)
	{
		
		buzzerSetNewFrequency(melody[0][noteIndex]);
		noteIndex++;
//			HAL_Delay(noteDurations[melodyIndex][noteIndex] * melodySlowfactor[melodyIndex]);

	}
//	__ValueStep(melodyIndex,1,melodyCount);
//   TIM9 -> CR1 &= ~TIM_CR1_CEN;
//   TIM9 ->CCER &= ~TIM_CCER_CC4E;
}
void music2Play()//¶ÂÈûÊ½
{
  int melodyCount = sizeof(melodySizes2)/ sizeof(uint32_t);

  for(int melodyIndex = 0; melodyIndex < melodyCount; melodyIndex++)
  {
	  for(int noteIndex = 0; noteIndex < melodySizes2[melodyIndex]; noteIndex++)
  	  {
	  	  buzzerSetNewFrequency(melody2[melodyIndex][noteIndex]);
	  	  HAL_Delay(noteDurations2[melodyIndex][noteIndex] * melodySlowfactor[melodyIndex]);
  	  }
  }

}
void music3Play()
{
	__ExecuteOnce(buzzerDriverInit());
	__ExecuteOnce(TIM9 -> CR1 |= TIM_CR1_CEN);
	__ExecuteOnce(TIM9 ->CCER |= TIM_CCER_CC4E);
	__ExecuteOnce(TIM12 -> CR1 |= TIM_CR1_CEN);
	__ExecuteOnce(TIM12 ->CCER |= TIM_CCER_CC4E);
	
	static uint32_t cnt_=0;
	static uint32_t cnt2_=0;
	
//  static int melodyCount = sizeof(melodySizes)/ sizeof(uint32_t);
//	static int melodyIndex = 0;
	static int noteIndex = 0;
	static int noteIndex2 = 0;
	
	if( noteIndex >= melodySizes3[0]) noteIndex=0;
	if( noteIndex2 >= melodySizes3_2[0]) noteIndex2=0;
	
	if(cnt_++%(noteDurations3[0][noteIndex] * melodySlowfactor[0])==0){
		buzzerSetNewFrequency(melody3twincle[0][noteIndex]);
		noteIndex++;
	}
	if(cnt2_++%(noteDurations3_2[0][noteIndex2] * melodySlowfactor[0])==0){
		buzzerSetNewFrequency2(melody3twincle2[0][noteIndex2]);
		noteIndex2++;
	}
//	__ValueStep(melodyIndex,1,melodyCount);
//   TIM9 -> CR1 &= ~TIM_CR1_CEN;
//   TIM9 ->CCER &= ~TIM_CCER_CC4E;
}
