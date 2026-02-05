#include "uart.h"
#include "string.h"

void UART_Receive_Enable(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size){
    HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
}
void UART_ResetPlotData(PlotBuffer_t *data){
		memset(data->buffer, 0, sizeof(data->buffer));
		data->count = 0;
		data->write_idx = 0;
		data->read_idx = 0;
}
