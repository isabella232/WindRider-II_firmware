//! FaulhaberComm declaration file.
/**
 * @file      FaulhaberComm.h
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 *
 */

#ifndef FAULHABERCOMM_H
#define FAULHABERCOMM_H

#include <algorithm>
#include <string>
#include <set>

#include "initialization.h"
#include "CommandQueue.h"

//! Faulhaber Communication class.
/**
 *  @brief This helper class serves as an abstraction layer for communication with Faulhaber motor drivers.
 *         An object of this class represents a seperate motor driver on rs-232 network.
 */
class FaulhaberComm{

    public:

    //! Default FaulhaberComm constructor.
    /**
     * @brief The default constructor. initialize_hardware() must be called after core mcu initialization.
     * @param addr Address of the initialized motor driver.
     */ 
    FaulhaberComm(const uint8_t addr);

    //! initialize_hardware method.
    /**
     * @brief Initializes the dedicated communication pheripheral (e.g. uart\rs-232), 
     *        then calls configure_motor_driver() for each instantiated motor driver object.
     */ 
    static void initialize_hardware(void);

    //! send method.
    /**
     * @brief A helper hardware abstraction method,
     *        sends a string via the dedicated communication pheripheral (e.g. uart\rs-232).
     * @param string String to send.
     */ 
    static void send(std::string string);

    //! write_sync method.
    /**
    * @brief Sends a command to both motors simmultaneously
    *        by temprorarily disabling asynchronous responses.
    * @param string String to send.
    */
    static void write_sync(std::string string);

    // Static methods to deal with uart driver and forwarding.

    //! process_feedback method.
    /**
    * @brief Called from the main loop to force print every terminated line received by uart.
    *        Used for debugging. Deletes the command form the queue after printing. 
    *        Call write_and_return for normal operation.
    */ 
    static void process_feedback(void);

    //! method enable_forwarding
    /**
    * @brief After calling this functions every line received by uart is printed. Sets _forwarding flag.
    */ 
    static void enable_forwarding(void);

    //! method disable_forwarding
    /**
    * @brief Disables the action of enable_forwarding. Resets _forwarding flag.
    */
    static void disable_forwarding(void);

    //! method forward_reply
    /**
    * @brief Forwards/Prints uart replies.
    * @param reply String to print.
    */
    static void forward_reply(const std::string reply);

    
    static void reset_receive_buffer(void);

    //! get_uart_handle method.
    /**
    * @brief Pass on uart handle to the STM HALibrary. Used in the interrupt handlers.
    * @return Handle to the uart instance.
    */
    static UART_HandleTypeDef* get_uart_handle(void);

    //! write_and_return function.
    /**
    * @brief Write and wait for a terminated reply in blocking mode.
    * @param cmd String to write.
    * @return reply.
    */
    const std::string write_and_return(std::string cmd);

    //! method write_and_confirm.
    /**
    * @brief Write and wait for acknowledgment.
    * @param cmd String to write.
    */
    void write_and_confirm(std::string cmd);


    //! configure_motor_driver method.
    /**
    * @brief Load default motor driver configuration.
    */ 
    void configure_motor_driver(void);

    //! set_velocity method.
    /**
    * @param velocity Set velocity.
    */
    void set_velocity(uint16_t velocity);

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