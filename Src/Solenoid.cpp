//! Solenoid implementation file.
/**
 * @file      Solenoid.cpp
 * @brief     Solenoid abstraction class.
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 */
#include "Solenoid.h"

// TODO: Mege windrider.cpp with initialization.cpp
// bad bad practices...
// Timer 3 (htim3) -- Generates pwm shared between suction motor and solenoid.
//                    Solenoids are not connected to the timer channels, thus have software defined pwm.
//                    Suction motor is connected to a dedicated pin and controlled through hardware pwm.
extern TIM_HandleTypeDef htim3;

//! Solenoid constructor.
/**
 * @brief Initializes a new solenoid instance, hardware must be configured throught calling initialize() command for each solenoid.
 *        Solenoids use TIM3 to generate square wave (0.5 duty cycle 2ms period) and count the on/off time.
 * @param GPIOx             Solenoid port.
 * @param GPIO_Pin          Solenoid pin.
 */
Solenoid::Solenoid(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin):
                    _GPIOx(GPIOx),_GPIO_Pin(GPIO_Pin){}

// Set of solenoids must be initialized
std::set<Solenoid*> Solenoid::_objects = {};

//! initialize method
/**
 * @brief Initializes gpio hardware to drive solenoids.
 */
void Solenoid::initialize(void){

        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
        // Configure GPIO pin Output Level
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        //HAL_GPIO_WritePin(const_cast<GPIO_TypeDef*>(_GPIOx), _GPIO_Pin, GPIO_PIN_SET);

        //Configure GPIO pins
        GPIO_InitStruct.Pin = _GPIO_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(const_cast<GPIO_TypeDef*>(_GPIOx), &GPIO_InitStruct);

        //HAL_GPIO_WritePin(const_cast<GPIO_TypeDef*>(_GPIOx), _GPIO_Pin, GPIO_PIN_SET);
        _initialized = true;
        __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

//! method on
/**
* @brief Starts the impactor with given pulse lenth and period.
* @param off_time_ms Delay between pulses in milliseconds.
* @param on_time_ms Delay of the impact puse in milliseconds.
*/ 
void Solenoid::on(uint16_t off_time_ms, uint16_t on_time_ms){
    
    if(_initialized){
        _off_time_ms = off_time_ms;
        _on_time_ms = on_time_ms;
        _objects.insert(this);
    }
        
}

//! method off
/**
* @brief Turns off the Impactor.
*/ 
void Solenoid::off(void){
    if(_initialized)
        _objects.erase(this);
}
// A Destructor just in case we need it.
Solenoid::~Solenoid(){

    HAL_GPIO_WritePin(const_cast<GPIO_TypeDef*>(this->_GPIOx), this->_GPIO_Pin, GPIO_PIN_RESET);
    this->off();
}


//! method tick.
/**
 * @brief Called from a timer interrupt to keep track of on/off and pulse timings for each Solenoid instance.
 */
void Solenoid::tick(void){

    std::for_each(_objects.begin(), _objects.end(),
                    [](Solenoid* solenoid){
                        
                        // Reload the counter if it ran out, change _active state
                        if(solenoid->_counter == 0){

                            if(solenoid->_active){

                                solenoid->_counter = solenoid->_off_time_ms;
                                HAL_GPIO_WritePin(const_cast<GPIO_TypeDef*>(solenoid->_GPIOx), solenoid->_GPIO_Pin, GPIO_PIN_SET);
                            }

                            else
                                solenoid->_counter = solenoid->_on_time_ms;

                            solenoid->_active = !solenoid->_active;
                        }
                        
                        // If solenoid is active toggle the solenoid pin.
                        if(solenoid->_active)
                            HAL_GPIO_TogglePin(const_cast<GPIO_TypeDef*>(solenoid->_GPIOx), solenoid->_GPIO_Pin);

                        --solenoid->_counter;
                    });
}