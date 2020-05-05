//! CommandParser declaration file.
/**
 * @file      UsbComm.h
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 *
 */
#ifndef USBCOMM_H
#define USBCOMM_H

#include <string>

#include "CommandQueue.h"

//! class UsbComm
/**
 * @brief A helper class to handle usb cdc communication. 
 */
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

#endif