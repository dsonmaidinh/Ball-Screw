#include "system_config.h"
#include "stm32f4xx.h"

#define MaxBuffer				100
#define PLOT_BUFFER_SIZE 700

typedef struct {
    float q_desired;
    float q_actual;
    float v_desired;
    float v_actual;
    float v_motor;
    float a_desired;
    float a_motor;
		float pwm;
		float time;
} PlotData_t;

typedef struct {
    PlotData_t buffer[PLOT_BUFFER_SIZE];
    volatile uint16_t write_idx;
    volatile uint16_t read_idx;
    volatile uint16_t count;
		uint8_t reset;
} PlotBuffer_t;

void UART_Receive_Enable(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void UART_ResetPlotData(PlotBuffer_t *data);
