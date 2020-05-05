#ifndef WINDRIDER_H
#define WINDRIDER_H

#include <set>
#include <array>
#include <deque>
#include <string>
#include <algorithm>

#include "usbd_cdc_if.h"
#include "CommandQueue.h"
//#include "FaulhaberComm.h"

class UsbComm {

    public:
    //! printf implementation
    /**
     * @brief This function sends a string over USB virtual com port.
     * @param str_to_send Pointer to the string to be sent.
     */
    static void usb_send(const std::string *str_to_send);

    // A Queue of commands seperated by '\r'
    static CommandQueue<std::string, '\r'> usb_queue;
};

namespace HardwareDriver {

    const std::string get_status(void);

    void initialize(void);

    // Command handlers
    void servo_angle(std::string arg);
    void suction_power(std::string arg);
    void solenoid(std::string arg);
    void led(std::string arg);
    void suction_current(std::string arg);
    void uart(std::string arg);
    void left(std::string arg);
    void right(std::string arg);
    void straight(std::string arg);
    

    void error(const std::string error);

    class Solenoid{

        public:
        // Do not allow default constructor.

        Solenoid(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
        
        ~Solenoid();

        void initialize(void);
        void adjust(void);
        void on(uint16_t off_time_ms, uint16_t on_time_ms);
        void off(void);

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
    
    const std::string response_ok = "ok\n\r";
    const std::string invalid_argument = "invalid argument\n\r";
    const std::string arguments_out_of_range = "arguments out of range\n\r";
    const std::string error_reset = "Runtime error! Reset required!\n\r";
};

#endif