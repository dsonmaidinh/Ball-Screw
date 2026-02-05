#include "encoder.h"
#include "stm32f4xx.h"
#include "system_config.h"

void ProcessEncoder(encoder *Enc, uint16_t PinA, uint16_t PinB) {
    uint8_t State = 0;
    
    // Đọc trạng thái encoder
    State = (State << 1) | HAL_GPIO_ReadPin(Encoder_Port, PinA);
    State = (State << 1) | HAL_GPIO_ReadPin(Encoder_Port, PinB);
    State &= 0x03;
    
    // Quan sát trạng thái - kiểm tra có lỗi encoder không?
    if (Enc->index_End < MAX_STATE) {
        Enc->EncState[Enc->index_End++] = State;
    }
    
    // Xác định hướng quay theo state machine
    switch (State) {
			case 0:
				if(Enc->PreviousState==1) Enc->CountValue++;
				else if(Enc->PreviousState == 2) Enc->CountValue--;
				break;
			case 1:
				if(Enc->PreviousState==3) Enc->CountValue++;
				else if(Enc->PreviousState == 0) Enc->CountValue--;
				break;
			case 2:
				if(Enc->PreviousState==0) Enc->CountValue++;
				else if(Enc->PreviousState == 3) Enc->CountValue--;
				break;
			case 3:
				if(Enc->PreviousState==2) Enc->CountValue++;
				else if(Enc->PreviousState == 1) Enc->CountValue--;
				break;
		}
    Enc->PreviousState = State;
    Enc->CntVel++;
    
    // Xử lý overflow/underflow
    if (Enc->CountValue >= Encoder_ppr) {
        Enc->CountValue = 0;
        Enc->PosCnt++;
    } else if (Enc->CountValue <= -Encoder_ppr) {
        Enc->CountValue = 0;
        Enc->PosCnt--;
    }
}
//void ProcessEncodeTIM(encoder *Enc, TIM_HandleTypeDef *htim) {
//    Enc->CntVel++;
//    Enc->CountValue = __HAL_TIM_GetCounter(htim);
//	
//    // Xử lý overflow/underflow
//    if (Enc->CountValue >= Encoder_ppr) {
//        Enc->CountValue = 0;
//        Enc->PosCnt++;
//    } else if (Enc->CountValue <= -Encoder_ppr) {
//        Enc->CountValue = 0;
//        Enc->PosCnt--;
//    }
//}
void ENCODER_ResetParams(encoder *Enc){
		Enc->PosCnt = 0;
		Enc->Cnttmp = 0;
		Enc->CountValue = 0;
		Enc->CntVel = 0;
		Enc->PreviousState = 0;
}
