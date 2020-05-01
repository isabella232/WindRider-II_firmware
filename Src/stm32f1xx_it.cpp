
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
*/

#include "initialization.h"
#include "stm32f1xx_it.h"
#include "windrider.h"
#include "FaulhaberComm.h"

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void){

}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void){

  while (1){
    // Oops...
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {

  while (1){

  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void){

  while (1){
    
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void){

  while (1){

  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void){

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void){

}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void){

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void){

  HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void){
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  
}
void USB_HP_CAN1_TX_IRQHandler(void){

  HAL_PCD_IRQHandler(&hpcd_USB_FS);

}
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void){

  // Check if we received anything before the shady HAL_UART_IRQHandler does it.
  if(FaulhaberComm::get_uart_handle()->Instance->SR & USART_SR_RXNE){

    const char received_byte = FaulhaberComm::get_uart_handle()->Instance->DR;
    const uint32_t len = 1;
    
    CLEAR_BIT(FaulhaberComm::get_uart_handle()->Instance->SR, USART_SR_RXNE);

    FaulhaberComm::feedback_queue.insert_received_elements(&received_byte, &len);

  }
  else if(FaulhaberComm::get_uart_handle()->Instance->SR & USART_SR_TC){
    __HAL_UART_DISABLE_IT(FaulhaberComm::get_uart_handle(), UART_IT_TC);

    /* Tx process is ended, restore huart->gState to Ready */
    FaulhaberComm::get_uart_handle()->gState = HAL_UART_STATE_READY;
  }

}

// USARTx_DMA_TX_IRQHandler
void DMA1_Channel4_IRQHandler(void){
  
  HAL_DMA_IRQHandler(FaulhaberComm::get_uart_handle()->hdmatx);
}

/**
  * @brief  This function handles ADC interrupt request.
  * @param  None
  * @retval None
  */
void ADCx_IRQHandler(void)
{
  //HAL_ADC_IRQHandler(&AdcHandle);
}

void TIM3_IRQHandler(void){

  HardwareDriver::Solenoid::tick();
  __HAL_TIM_CLEAR_IT(&htim3 ,TIM_IT_UPDATE);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
