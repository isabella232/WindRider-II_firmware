#ifndef FAULHABERCOMM_H
#define FAULHABERCOMM_H

#include <algorithm>
#include <string>
#include <set>

#include "initialization.h"
#include "CommandQueue.h"

class FaulhaberComm{

    public:

    // Initializations.
    FaulhaberComm(uint8_t addr);
    static void initialize();

    // Backend communications with motor drivers.
    static void send(std::string string);
    const std::string write_and_return(std::string cmd);
    void write_and_confirm(std::string cmd);
    std::string write_and_return(std::string *string);
    static void write_sync(std::string string);

    // Frontend communications with motor drivers.
    void configure_motor_driver(void);
    void set_velocity(uint16_t velocity);

    // Static functions to deal with uart driver and forwarding.
    static void process_feedback(void);
    static void enable_forwarding(void);
    static void disable_forwarding(void);
    static void forward_reply(const std::string reply);
    static void reset_receive_buffer(void);
    static UART_HandleTypeDef* get_uart_handle(void);

    // Terminated command queue.
    static CommandQueue<std::string, '\r'> feedback_queue;

    private:

    static std::set<FaulhaberComm*> _motors;

    static std::string _msg_to_send;

    const uint8_t _addr;

    static const std::string _terminator;
    static const std::string _failed_to_confirm;

    // Hardware handles.
    static UART_HandleTypeDef _huart;
    static DMA_HandleTypeDef _hdma_tx;

    // Flags.
    static bool _forwarding;
    static bool _initialized;

    // User Configuration.
    static const uint16_t _baud = 9600;
    static const uint16_t _max_travel_speed = 2000;
    static const uint16_t _max_accel = 100;
};

#endif