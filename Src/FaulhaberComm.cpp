#include "FaulhaberComm.h"
#include "windrider.h"

// Check if hardware is initialized.
bool FaulhaberComm::_initialized = false;
// If forwarding enabled all responses are printed.
bool FaulhaberComm::_forwarding = false;

// This container is used by the dma to copy data to uart.
std::string FaulhaberComm::_msg_to_send = {};

// String constants.
const std::string FaulhaberComm::_terminator = "\n\r";
const std::string FaulhaberComm::_failed_to_confirm = "Faulhaber Failed to Confirm";

// Hardware handles for interrupt handlers.
UART_HandleTypeDef FaulhaberComm::_huart = {};
DMA_HandleTypeDef FaulhaberComm::_hdma_tx = {};

// Set of all motor objects.
std::set<FaulhaberComm*> FaulhaberComm::_motors = {};

// Queue of commands received by uart. Not really usefull without an operating system.
CommandQueue<std::string, '\r'> FaulhaberComm::feedback_queue = {};

// Initialize a motor driver with a given address
FaulhaberComm::FaulhaberComm(uint8_t addr):_addr(addr){

    _motors.insert(this);
};

//! method initialize
    /**
    * @brief Initializes hardware and loads configureations.
    */
void FaulhaberComm::initialize_hardware(){

    // Enable Clock
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure UART
    _huart.Instance = USART1;
    _huart.Init.BaudRate = _baud;
    _huart.Init.WordLength = UART_WORDLENGTH_8B;
    _huart.Init.StopBits = UART_STOPBITS_1;
    _huart.Init.Parity = UART_PARITY_NONE;
    _huart.Init.Mode = UART_MODE_TX_RX;
    _huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    _huart.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&_huart);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /**USART1 GPIO Configuration    
    PA9   ------> USART1_TX
    PA10  ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure the DMA handler for Transmission process.
    _hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
    _hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    _hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    _hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    _hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    _hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    _hdma_tx.Init.Mode                = DMA_NORMAL;
    _hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&_hdma_tx);

    // Associate the initialized DMA handle to the UART handle.
    __HAL_LINKDMA(&_huart, hdmatx, _hdma_tx);

    // Enable receive interrupt.
    __HAL_UART_ENABLE_IT(&_huart, UART_IT_RXNE);

    _initialized = true;

    // NVIC configuration for DMA transfer complete interrupt (USART1_TX).
    HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);

    // NVIC configuration for all UART interrupts.
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    std::for_each(_motors.begin(), _motors.end(), 
                  [](FaulhaberComm* motor){
                        motor->configure_motor_driver();
                  });

};

//! method configure_motor_driver
    /**
    * @brief Load default motor driver configuration.
    */ 
void FaulhaberComm::configure_motor_driver(void){

    if(!_initialized)
      HardwareDriver::error("FaulhaberComm::initialize() must be called before any object instantiation\n\r");

    // Load travel speed.
    write_and_confirm("SP" + std::to_string(_max_travel_speed));

    // Load acceleration.
    write_and_confirm("AC" + std::to_string(_max_accel));
    write_and_confirm("DEC" + std::to_string(_max_accel));

    // Enable Drive.
    write_and_confirm("EN");
};

//! method send
    /**
    * @brief Sends a string over uart in a non blocking mode. Waits until uart's dma is available.
    * @param cmd String to send.
    */ 
void FaulhaberComm::send(std::string cmd){

    // Sandwich node address and line termination into a static class member that won't get optimized out.
    _msg_to_send = cmd;

    if(_msg_to_send.back() != '\r')
        _msg_to_send += '\r';

    while(HAL_OK != HAL_UART_Transmit_DMA(&_huart, reinterpret_cast<uint8_t*>(const_cast<char*>(_msg_to_send.data())), _msg_to_send.size())){};

};

//! method process_feedback
/**
* @brief Called from the main loop to force print every terminated line received by uart.
*        Used for debugging. Deletes the command form the queue after printing. Call write_and_return for normal operation.
*/ 
void FaulhaberComm::process_feedback(void){
    
    if(feedback_queue.get_queue_size() != 0){

        if(_forwarding)
            forward_reply(feedback_queue.get_next_cmd());

        feedback_queue.free_pending_cmd();
    }
};

//! method enable_forwarding
/**
* @brief After calling this functions every line received by uart is printed.
*/ 
void FaulhaberComm::enable_forwarding(void){
    _forwarding = true;
};

//! method disable_forwarding
    /**
    * @brief Disables the action of enable_forwarding.
    */
void FaulhaberComm::disable_forwarding(void){
    _forwarding = false;
};

//! method forward_reply
    /**
    * @brief Forwards/Prints uart replies.
    * @param reply to print.
    */
void FaulhaberComm::forward_reply(const std::string reply) {

    const auto uart_says = "uart: " + reply + "\n\r";
    UsbComm::usb_send(&uart_says);
};

//! function write_and_return.
    /**
    * @brief Write and wait for a terminated reply in blocking mode.
    * @param cmd to write.
    * @return reply.
    */
const std::string FaulhaberComm::write_and_return(std::string cmd){

    send(std::to_string(_addr) + cmd + "\n\r");

    // Wait for reply
    while(feedback_queue.get_queue_size() == 0){};

    const auto reply = feedback_queue.get_next_cmd();

    if(_forwarding)
        forward_reply(reply);

    return reply;
};

//! method write_and_confirm.
    /**
    * @brief Write and wait for acknowledgment.
    * @param cmd to write.
    */
void FaulhaberComm::write_and_confirm(std::string cmd){

    const auto response = write_and_return(cmd);
    if("OK\n\r" != response)
        forward_reply(_failed_to_confirm);
};

//! method write_sync.
    /**
    * @brief Sends a command to both motors simmultaneously
    *        by temprorarily disabling asynchronous responses
    *        to prevent serial from crashing.
    * @param cmd to write.
    */
void FaulhaberComm::write_sync(std::string cmd){

    send("ANSW0");
    
    send(cmd);

    std::for_each(_motors.begin(), _motors.end(), 
                  [](FaulhaberComm* motor){

                      motor->write_and_confirm("ANSW2");
                  });

}
//! set_velocity method.
    /**
    * @param velocity Set velocity.
    */
void FaulhaberComm::set_velocity(uint16_t velocity){

    write_and_confirm("V" + std::to_string(velocity));
};

//! method get_uart_handle.
    /**
    * @brief Pass on uart handle to the STM HALibrary. Used in the interrupt handlers.
    * @return Handle to the uart instance.
    */
UART_HandleTypeDef* FaulhaberComm::get_uart_handle(void){

    return &_huart;
};