//! Solenoid declaration file.
/**
 * @file      Solenoid.h
 * @brief     This solenoid abstraction class.
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 */

#ifndef SOLENOID_H
#define SOLENOID_H

#include "stm32f1xx_hal.h"

#include <set>
#include <algorithm>
//! Solenoid class
/**
 * @brief This is a driver class for a solenoid. 
 *        WARNING: In the current implementation, solenoids use a software defined pwm
 *                 and share a Timer 3 with suction motor.
 */
class Solenoid{

    public:
    //! Solenoid constructor.
    /**
     * @brief Initializes a new solenoid instance, hardware must be configured throught calling initialize() command for each solenoid.
     *        Solenoids use TIM3 to generate square wave (0.5 duty cycle 2ms period) and count the on/off time.
     * @param GPIOx             Solenoid port.
     * @param GPIO_Pin          Solenoid pin.
     */
    Solenoid(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    
    ~Solenoid();
    //! initialize method
    /**
     * @brief Initializes gpio hardware to drive solenoids.
     */
    void initialize(void);

    void configure_timings(uint16_t off_time_ms, uint16_t on_time_ms);

    //! method on
    /**
    * @brief Starts the impactor with given pulse lenth and period.
    * @param off_time_ms Delay between pulses in milliseconds.
    * @param on_time_ms Delay of the impact puse in milliseconds.
    */ 
    void on(void);

    //! method off
    /**
    * @brief Turns off the Impactor.
    */ 
    void off(void);

    //! method tick.
    /**
     * @brief Called from a timer interrupt to keep track of on/off and pulse timings for each Solenoid instance.
     */
    static void tick(void);

    private:
    
    // To control all solenoids we need to keep a list of 
    // objects that were ever created.
    static std::set<Solenoid*> _objects;

    // Solenoid's port and pin.
    const GPIO_TypeDef *_GPIOx;
    const uint16_t _GPIO_Pin;

    uint16_t _on_time_ms; // [ms]
    uint16_t _off_time_ms; // [ms]

    uint16_t _counter = 0;
    bool _active = false;
    bool _initialized = false;
};

#endif