#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "stm32f4xx.h"

#define MAX_STATE 8
typedef struct {
    int16_t EncState[MAX_STATE];
		uint8_t index_End;
		int32_t PosCnt;
		int32_t Cnttmp;
		int16_t CountValue;
		int16_t CntVel;
		uint8_t PreviousState;
} encoder;

void ProcessEncoder(encoder *Enc, uint16_t PinA, uint16_t PinB);
void ProcessEncodeTIM(encoder *Enc, TIM_HandleTypeDef *htim);
void ENCODER_ResetParams(encoder *Enc);

#endif
