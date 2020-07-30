#include "UsbComm.h"
#include "usbd_cdc_if.h"

//! printf implementation
/**
 * @brief This function sends a string over USB virtual com port.
 * @param str_to_send Pointer to the string to be sent.
 */
void UsbComm::usb_send(const std::string &str_to_send){

    uint8_t *buff_ptr = reinterpret_cast<uint8_t*>(const_cast<char*>(str_to_send.data()));

    while(CDC_Transmit_FS(buff_ptr, static_cast<uint16_t>(str_to_send.length())) != USBD_OK){}
};

CommandQueue<std::string, '\r'> UsbComm::usb_queue;
