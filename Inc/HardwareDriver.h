//! HardwareDriver declaration file.
/**
 * @file      HardwareDriver.h
 * @brief     This file contains:
 *              - Peripheral initializations.
 *              - Command handlers.
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 */
#ifndef WINDRIDER_H
#define WINDRIDER_H

#include <set>
#include <array>
#include <deque>
#include <string>
#include <algorithm>

namespace HardwareDriver {

    //! method get_status
    /**
    * @return status from execution of the previous command..
    */
    const std::string get_status(void);

    void initialize(void);

    // Command handlers

    //! method servo_angle
    /**
    * @brief Handler for servo command. Sets servo to a given angle.
    * @param arg Argument string. Comes after token.
    */ 
    void servo_angle(std::string arg);

    //! method suction_power
    /**
    * @brief Handler for suction command. Adjusts suction power [0-100]
    * @param arg Argument string. Comes after token.
    */ 
    void suction_power(std::string arg);


    void solenoid(std::string arg);
    void led(std::string arg);
    void suction_current(std::string arg);
    void uart(std::string arg);
    void left(std::string arg);
    void right(std::string arg);
    void straight(std::string arg);
    
    //! method error
    /**
    * @param error Prints the content before killing execution.
    * @brief Used to troubleshoot errors that potentially cause hard faults.
    *        Kills/Resets the microcontroller.
    */ 
    void error(const std::string error);
    
    const std::string response_ok = "ok\n\r";
    const std::string invalid_argument = "invalid argument\n\r";
    const std::string arguments_out_of_range = "arguments out of range\n\r";
    const std::string error_reset = "Runtime error! Reset required!\n\r";
};

#endif