
// This template shows how to use the UART Callback in the HAL to not interrupt the "task"
// until something is in the UART Rx Fifo

#include "semphr.h"
#define UART_INT_PRIORITY 7

SemaphoreHandle_t uartSemaphore;

void uartCallback(void *callback_arg, cyhal_uart_event_t event)
{
  BaseType_t xHigherPriorityTaskWoken;
  cyhal_uart_enable_event(&cy_retarget_io_uart_obj,CYHAL_UART_IRQ_RX_NOT_EMPTY,UART_INT_PRIORITY,false);
  xSemaphoreGiveFromISR(uartSemaphore,&xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void uartTask(void *arg)
{
  setvbuf(stdin, NULL, _IONBF, 0);

  uartSemaphore = xSemaphoreCreateCounting(5,0); // Max=5 - Initial=0

  cyhal_uart_register_callback(&cy_retarget_io_uart_obj, uartCallback, 0);
  cyhal_uart_enable_event(&cy_retarget_io_uart_obj,CYHAL_UART_IRQ_RX_NOT_EMPTY,UART_INT_PRIORITY,true);

  unsigned char c;
  while(1)
    {
      xSemaphoreTake( uartSemaphore,0xFFFFFFF );
      cyhal_uart_getc(&cy_retarget_io_uart_obj, &c, 10);
      printf("C=%c\n",c);
      Cy_SCB_ClearRxInterrupt(cy_retarget_io_uart_obj.base, CY_SCB_RX_INTR_NOT_EMPTY);
      cyhal_uart_enable_event(&cy_retarget_io_uart_obj,CYHAL_UART_IRQ_RX_NOT_EMPTY,UART_INT_PRIORITY,true);
    }
}


// In main
// xTaskCreate(uartTask, "uartTask", configMINIMAL_STACK_SIZE,0 /* args */ ,1 /* priority */, 0 /* blinkTaskHandle */);
