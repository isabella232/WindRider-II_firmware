#include "windrider.h"
#include "FaulhaberComm.h"
#include "initialization.h"

// TODO: Mege windrider.cpp with initialization.cpp
// bad bad practices...
extern TIM_HandleTypeDef htim2, htim3;
extern ADC_HandleTypeDef hadc1;

//! printf implementation
/**
 * @brief This function sends a string over USB virtual com port.
 * @param str_to_send Pointer to the string to be sent.
 */
void UsbComm::usb_send(const std::string *str_to_send){

    uint8_t *buff_ptr = reinterpret_cast<uint8_t*>(const_cast<char*>(str_to_send->data()));

    CDC_Transmit_FS(buff_ptr, static_cast<uint16_t>(str_to_send->length()));
};
CommandQueue<std::string, '\r'> UsbComm::usb_queue;

//! namespace HardwareDriver
/**
 * @brief This namespace contains abstracted hardware level command handlers.
 */
namespace HardwareDriver {

    // We'll use this string to save the status of the command execution.
    std::string status = {};

    FaulhaberComm right_motor(1), left_motor(2);

    // Windrider is currently capable of driving 2 solenoid channels.
    std::array<Solenoid, 2> solenoids = {
        Solenoid(SOLENOID_GPIO_Port, SOLENOID_Pin),
        Solenoid(SPARE_GPIO_Port, SPARE_Pin)
    };

    
    //! method initialize
    /**
    * @brief Configure hardware (gpio) for each solenoid.
    */  
    void initialize(void){

        std::for_each(solenoids.begin(), solenoids.end(), 
                    [](Solenoid &solenoid){
                        solenoid.initialize();
                    });

        //FaulhaberComm::initialize();
    }

    //! method error
    /**
    * @param error Prints the content before killing execution.
    * @brief Used to troubleshoot errors that potentially cause hard faults.
    *        Kills/Resets the microcontroller.
    */ 
    void error(const std::string error){

        UsbComm::usb_send(&error);
        UsbComm::usb_send(&error_reset);
        
        // Kills program execution.
        throw std::runtime_error("Oops...");
    }
    
    //! method get_status
    /**
    * @return status from execution of the previous command..
    */
    const std::string get_status(void){
        return status;
    }

    //! method servo_angle
    /**
    * @brief Handler for servo command.
    */ 
    void servo_angle(std::string arg){

        const auto separator = arg.find_first_of(' ');

        if(separator == std::string::npos){
            status = invalid_argument;
            return;
        }

        auto channel = 0, angle = 0;

        // Returns zero on failure
        if(std::isdigit(arg.at(0)) and std::isdigit(arg.at(separator + 1))){

            channel = std::stoul(arg.substr(0, separator));
            angle = std::stoul(arg.substr(separator));
        }
        else{

            status = invalid_argument;
            return;
        }

        if(0 <= angle and angle <= 180){ 

            // Enable channels
            TIM_OC_InitTypeDef sConfigOC = {};

            sConfigOC.OCMode = TIM_OCMODE_PWM1;
            sConfigOC.Pulse = static_cast<uint16_t>( htim2.Init.Period*(0.05 + 0.05*angle/180.0) );
            sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; 
            sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

            if(channel == 0){
                HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
            }
            else if(channel == 1){
                HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
            }
            else
                status = arguments_out_of_range;
            
            status = response_ok;
        }
        else
            status = arguments_out_of_range;
    }

    //! method suction_power
    /**
    * @brief Handler for suction command.
    */ 
    void suction_power(std::string arg){
        
        TIM_OC_InitTypeDef sConfigOC = {0};

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;


        if(arg[0] == '0'){

            sConfigOC.Pulse = 0;
            HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

            status = response_ok;

            return;
        }

        // on/off    
        if(arg == "on\r"){
            HAL_GPIO_WritePin(SUCTION_EN_GPIO_Port, SUCTION_EN_Pin, GPIO_PIN_RESET);
            status = response_ok;
            return;
        }
        else if(arg == "off\r"){

            // PWM to Zero
            sConfigOC.Pulse = 0;
            HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

            // Unplug power
            HAL_GPIO_WritePin(SUCTION_EN_GPIO_Port, SUCTION_EN_Pin, GPIO_PIN_SET);
            status = response_ok;
            return;
        }
        
        auto power = 0;

        if(std::isdigit(arg.at(0)))
            power = std::stoul(arg);

        else{
            
        }
        
        if(0 <= power and power <= 100){
            sConfigOC.Pulse = static_cast<uint16_t>(htim3.Init.Period*power/100.0);
            HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

            status = response_ok;
        }

        else{
            status = invalid_argument;
        }
    }
    
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

    // This function parses command line arguments for solenoid token
    void solenoid(std::string arg){

        const auto separator1 = arg.find_first_of(' ');

        // At least 2 arguments required.
        if(separator1 == std::string::npos){
            status = invalid_argument;
            return;
        }

        auto separator2 = arg.substr(separator1 + 1).find_first_of(' ');

        // Figure out which channel.
        auto channel = 0;
        if(std::isdigit(arg.at(0))){

            channel = std::stoul(arg.substr(0, separator1));

            if(channel >= solenoids.size()){
                
                status = arguments_out_of_range;
                return;
            }
        }
        else{

            status = invalid_argument;
            return;
        }

        if(separator2 == std::string::npos){

            const auto state = arg.substr(separator1 + 1);

            if(state == "off\r")
                solenoids.at(channel).off();
        }
        else{
            separator2 += separator1 + 1;

            auto on_time_ms = 0, off_time_ms = 0;

            if(std::isdigit(arg.at(separator1 + 1)) and std::isdigit(arg.at(separator2 + 1))){

                off_time_ms = std::stoul(arg.substr(separator1, separator2 - separator1));
                on_time_ms = std::stoul(arg.substr(separator2));
            }
            else{

                status = invalid_argument;
                return;
            }

            solenoids.at(channel).on(off_time_ms, on_time_ms);
        }

        status = response_ok;
    }

    // Handler for uart command.
    void led(std::string arg){

        const auto separator = arg.find_first_of(' ');

        if(separator == std::string::npos){
            status = invalid_argument;
            return;
        }

        auto channel = 0, current = 0;

        if(std::isdigit(arg.at(0)) and std::isdigit(arg.at(separator + 1))){

            channel = std::stoul(arg.substr(0, separator));
            current = std::stoul(arg.substr(separator));
        }
        else {

            status = invalid_argument;
            return;
        }
        
        
        // Check if current value is sane.
        if(0 <= current <= 1500){

            // Enable channels
            TIM_OC_InitTypeDef sConfigOC = {};

            sConfigOC.OCMode = TIM_OCMODE_PWM1;
            sConfigOC.Pulse = static_cast<uint16_t>( htim2.Init.Period*( current/1500.0 ) );
            sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; 
            sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

            if(channel == 0){

                HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            }
            else if(channel == 1){
                
                HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
            }
            status = response_ok;
            return;
        }
        else{
            status = arguments_out_of_range;
            return;
        }

        status = invalid_argument;
    }

    
    void suction_current(std::string arg){

        const auto adc_value = HAL_ADC_GetValue(&hadc1);

        status = std::to_string(adc_value) + "\n\r";
    }

    // Handler for uart command.
    void uart(std::string arg){

        const auto separator = arg.find_first_of(' ');

        if(separator == std::string::npos){
            status = invalid_argument;
            return;
        }

        else if(arg.substr(0, separator) == "forward"){
            
            FaulhaberComm::send(arg.substr(separator + 1));
            status = response_ok;
            return;
        }
        else if(arg.substr(0, separator) == "reply"){

            if(arg.substr(separator + 1) == "on\r")
                FaulhaberComm::enable_forwarding();

            else if(arg.substr(separator + 1) == "off\r")
                FaulhaberComm::disable_forwarding();

            else{

                status = invalid_argument;
                return;
            }
            status = response_ok;
        }
        else
            status = invalid_argument;
    }

    // Handler for left command.
    void left(std::string arg){

        if(std::isdigit(arg.at(0)) or arg.at(0) == '-'){

            const auto velocity = std::stoi(arg);

            left_motor.set_velocity(velocity);

            status = response_ok;
        }
        else
            status = invalid_argument;
        
    }

    // Handler for right command.
    void right(std::string arg){

        if(std::isdigit(arg.at(0)) or arg.at(0) == '-'){

            const auto velocity = std::stoi(arg);

            right_motor.set_velocity(velocity);

            status = response_ok;
        }
        else
            status = invalid_argument;

    }

    void straight(std::string arg){

        if(std::isdigit(arg.at(0)) or arg.at(0) == '-'){

            const auto velocity = std::stoi(arg);

            FaulhaberComm::write_sync("V" + std::to_string(velocity));

            status = response_ok;
        }
        else
            status = invalid_argument;

    }
}

