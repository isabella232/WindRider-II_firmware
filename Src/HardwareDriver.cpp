//! HardwareDriver implementation file.
/**
 * @file      HardwareDriver.cpp
 * @brief     This file contains:
 *              - Peripheral initializations.
 *              - Command handlers.
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 *
 */
#include "HardwareDriver.h"
#include "FaulhaberComm.h"
#include "initialization.h"
#include "Solenoid.h"
#include "UsbComm.h"

// TODO: Mege windrider.cpp with initialization.cpp
// bad bad practices...
// Timer 3 (htim3) -- Generates pwm shared between suction motor and solenoid.
//                    Solenoids are not connected to the timer channels, thus have software defined pwm.
//                    Suction motor is connected to a dedicated pin and controlled through hardware pwm.
// Timer 2 (htim2) --  Generates pwm shared between led drivers and servos. All hardware pwm.
extern TIM_HandleTypeDef htim2, htim3;

// ADC is used to measure suction current.
// Currently does not work due to a hardware related issue.
extern ADC_HandleTypeDef hadc1;

//! namespace HardwareDriver
/**
 * @brief This namespace contains abstracted hardware level command handlers.
 */
namespace HardwareDriver {

    // We'll use this string to save the status of the command execution.
    std::string status = {};

    FaulhaberComm right_motor(1), left_motor(2);

    std::array<uint16_t, 2> led_current = {0, 0};

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
        // Faulhaber control is implemented from the host computer.
        // FaulhaberComm::initialize_hardware();
    }

    //! method error
    /**
    * @param error Prints the content before killing execution.
    * @brief Used to troubleshoot errors that potentially cause hard faults.
    *        Kills/Resets the microcontroller.
    */ 
    void error(const std::string error){

        UsbComm::usb_send(error);
        UsbComm::usb_send(error_reset);
        
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
    void servo_angle(const std::vector<std::string> &args){

        // Sainity checks.
        if(args.size() != 2){

            status = invalid_argument;
            return;
        }
        
        //! Check if all of the characters in the args are digits
        /**
         * Ideally std::stoul() would throw an exception,
         * though we are using the nano version of C++ stl,
         * which does not support exceptions, a thrown exception will
         * end up in the terminate() handler 
         */ 
        auto first_arg_is_digit = std::all_of(args.front().begin(), 
                                              args.front().end(), 
                                              static_cast<int(*)(int)>(std::isdigit));

        auto second_arg_is_digit = std::all_of(args.at(1).begin(), 
                                              args.at(1).end(), 
                                              static_cast<int(*)(int)>(std::isdigit));

        if(not first_arg_is_digit and not second_arg_is_digit){

            status = invalid_argument;
            return;
        }

        auto channel = std::stoul(args.front());
        auto angle = std::stoul(args.at(1));

        if(not (0 <= angle <= 180 and 0 <= channel <= 1)){ 

            status = arguments_out_of_range;
            return;
        }

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

    //! method suction_power
    /**
    * @brief Handler for suction command.
    */ 
    void suction_power(const std::vector<std::string> &args){
        
        if(args.size() != 1){

            status = invalid_argument;
            return;
        }

        // on/off    
        if(args.front() == "on"){
            HAL_GPIO_WritePin(SUCTION_EN_GPIO_Port, SUCTION_EN_Pin, GPIO_PIN_RESET);
            status = response_ok;
            return;
        }

        TIM_OC_InitTypeDef sConfigOC = {0};

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        if(args.front() == "off"){

            // PWM to Zero
            sConfigOC.Pulse = 0;
            HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

            // Unplug power
            HAL_GPIO_WritePin(SUCTION_EN_GPIO_Port, SUCTION_EN_Pin, GPIO_PIN_SET);
            status = response_ok;
            return;
        }
        
        auto first_arg_is_digit = std::all_of(args.front().begin(), 
                                        args.front().end(), 
                                        static_cast<int(*)(int)>(std::isdigit));

        if(not first_arg_is_digit) {

            status = invalid_argument;
            return;
        }
        auto power = std::stoul(args.front());
        
        if(not (0 <= power <= 100)){

            status = arguments_out_of_range;
            return;
        }

        sConfigOC.Pulse = static_cast<uint16_t>(htim3.Init.Period*power/100.0);
        HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

        status = response_ok;
    }

    // This function parses command line arguments for solenoid token
    void solenoid(const std::vector<std::string> &args) {

        // Sainity checks.
        if(args.size() == 1 or args.size() > 3){
 
            status = invalid_argument;
            return;
        }

        auto first_arg_is_digit = std::all_of(args.front().begin(), 
                                        args.front().end(), 
                                        static_cast<int(*)(int)>(std::isdigit));

        if(not first_arg_is_digit) {

            status = invalid_argument;
            return;
        }
 
        auto channel = std::stoul(args.front());

        if(channel >= solenoids.size()){
            
            status = arguments_out_of_range;
            return;
        }

        if(args.at(1) == "off"){
            
            solenoids.at(channel).off();
            return;
        }

        if(args.at(1) == "on"){

            solenoids.at(channel).on();
            return;
        }

        if(args.size() == 2){
            
            status = invalid_argument;
            return;
        }

        auto second_arg_is_digit = std::all_of(args.at(1).begin(), 
                                              args.at(1).end(), 
                                              static_cast<int(*)(int)>(std::isdigit));

        auto third_arg_is_digit = std::all_of(args.at(2).begin(), 
                                              args.at(2).end(), 
                                              static_cast<int(*)(int)>(std::isdigit));

        if(not second_arg_is_digit or not third_arg_is_digit){
                
            status = invalid_argument;
            return;
        }

        auto off_time_ms = std::stoul(args.at(1));
        auto on_time_ms = std::stoul(args.at(2));

        solenoids.at(channel).configure_timings(off_time_ms, on_time_ms);

        status = response_ok;
    }

    // Handler for uart command.
    void led(const std::vector<std::string> &args){

        // Sainity checks.
        if(args.size() != 2){
 
            status = invalid_argument;
            return;
        }
        
        //! Check if all of the characters in the args are digits
        /**
         * Ideally std::stoul() would throw an exception,
         * though we are using the nano version of C++ stl,
         * which does not support exceptions, a thrown exception will
         * end up in the terminate() handler 
         */ 
        auto first_arg_is_digit = std::all_of(args.front().begin(), 
                                              args.front().end(), 
                                              static_cast<int(*)(int)>(std::isdigit));
        if(not first_arg_is_digit){
            
            status = invalid_argument;
            return;
        }

        // Convert argument strings to integers.
        auto channel = std::stoul(args.front());

        if(not (0 <= channel <= 1)){

            status = arguments_out_of_range;
            return;
        }
         

        if(args.at(1) == "off"){

            if(channel == 0)
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

            else if(channel == 1)
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

            status = response_ok;
            return;
        }
        if(args.at(1) == "on"){
            
            if(led_current.at(channel) > 0){
            
                if(channel == 0)
                    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

                else if(channel == 1)
                    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
            }
            status = response_ok;
            return;
        }

        auto second_arg_is_digit = std::all_of(args.at(1).begin(), 
                                              args.at(1).end(), 
                                              static_cast<int(*)(int)>(std::isdigit));

        if(not second_arg_is_digit){
            
            status = invalid_argument;
            return;
        }
 
        // Convert argument strings to integers.
        auto current = std::stoul(args.at(1));

        // Check if current value is sane.
        if(not (0 <= current <= 1500)){

            status = arguments_out_of_range;
            return;
        }

        led_current.at(channel) = current;

        // Enable channels
        TIM_OC_InitTypeDef sConfigOC = {};

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = static_cast<uint16_t>( htim2.Init.Period*( led_current.at(channel)/1500.0 ) );
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; 
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        
        if(channel == 0)
            HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

        if(channel == 1)
            HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

        status = response_ok;
    }

    void suction_current(const std::vector<std::string> &args){

        const auto adc_value = HAL_ADC_GetValue(&hadc1);

        status = std::to_string(adc_value) + "\n\r";
    }

    // Handler for uart command.
    void uart(const std::vector<std::string> &args){
       
       // Sainity checks.
        if(args.size() != 2){
 
            status = invalid_argument;
            return;
        }

        if(args.front() == "forward"){
            
            FaulhaberComm::send(args.at(1));
            status = response_ok;
            return;
        }
        else if(args.front() == "reply"){

            if(args.at(1) == "on")
                FaulhaberComm::enable_forwarding();

            else if(args.at(1) == "off")
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

    void move(const std::vector<std::string> &args){

        // Sainity checks.
        if(args.size() != 2){
 
            status = invalid_argument;
            return;
        }

        const auto first_arg_is_digit = std::all_of(args.front().begin(), 
                                                    args.front().end(), 
                                                    [](char c){

                                                        return std::isdigit(c) or c == '-';
                                                    });

        const auto second_arg_is_digit = std::all_of(args.at(1).begin(), 
                                                     args.at(1).end(), 
                                                     [](char c){
                                                  
                                                        return std::isdigit(c) or c == '-';
                                                    });

        if(not first_arg_is_digit or not second_arg_is_digit){

            status = invalid_argument;
            return;
        }

        const auto right_velocity = std::stoi(args.front());
        const auto left_velocity = std::stoi(args.at(1));

        right_motor.set_velocity(right_velocity);
        left_motor.set_velocity(left_velocity);

        status = response_ok;
    }

    void sync_straight(const std::vector<std::string> &args){

        // Sainity checks.
        if(args.size() != 1){
 
            status = invalid_argument;
            return;
        }

        const auto first_arg_is_digit = std::all_of(args.front().begin(), 
                                              args.front().end(), 
                                              [](char c){

                                                  return std::isdigit(c) or c == '-';
                                              });
        if(not first_arg_is_digit){

            status = invalid_argument;
            return;
        }

        const auto velocity = std::stoi(args.front());

        FaulhaberComm::write_sync("V" + std::to_string(velocity));

        status = response_ok;

    }


}

