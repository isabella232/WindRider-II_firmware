#ifndef COMMANDQUEUE_H
#define COMMANDQUEUE_H

#include <deque>

//! Template class CommandQueue
/**
 * @tparam Container        To store each command in the queue.
 * @tparam control_element  Software control element that seperates commands in the queue.
 * 
 */
template<class Container, 
         typename Container::value_type control_element>
class CommandQueue{

    public:

    CommandQueue() = default;

    //! insert_received_elements
    /**
     * @brief This function copies received elements into the buffer queue.
     *        It implements software flow control using control_element to separate commands in the queue.
     *        Called from CDC_Receive_FS.
     * @param Buf Buffer of data to be received.
     * @param Len Number of data received (in bytes)
     */
    void insert_received_elements(const typename Container::value_type* Buf, const uint32_t *Len){

        for(uint32_t i=0; i<*Len; ++i){
            
            if(_command_queue.empty() or _command_queue.back().back() == control_element)
                _command_queue.push_back(std::string(1, Buf[i]));
            else{
                _command_queue.back().push_back(Buf[i]);

                if(_command_queue.back().back() == control_element)
                    _command_queue.back().shrink_to_fit();
            }
        }  
    };

    void insert_received_elements(typename Container::value_type element){
   
        if(_command_queue.empty() or _command_queue.back().back() == control_element){
            _command_queue.push_back(Container());
            _command_queue.back().push_back(element);
        }
        else{
            _command_queue.back().push_back(element);

            if(_command_queue.back().back() == control_element)
                _command_queue.back().shrink_to_fit();

        }  
    };

    //! get_queue_size
    /**
     * @brief Returns the current usb queue size.
     * @return usb queue size
     */
    size_t get_queue_size(void){

        if(!_command_queue.empty() and _command_queue.back().back() != control_element)
            return _command_queue.size() - 1;
        
        return _command_queue.size();
    };

    //! get_next_cmd
    /**
     * @brief   Get the next command from the buffer. Keeps the command until free_pending_cmd called.
     * @return  Reference to the next command in the buffer.
     */
    const Container &get_next_cmd(void){

        return _command_queue.front();  
    };

    //! free_pending_cmd
    /**
     * @brief   After processing the command release it from the buffer.
     */
    void free_pending_cmd(void){

        if(!_command_queue.empty())
            _command_queue.pop_front();
    };

    private:

    std::deque<Container> _command_queue = {};
};

#endif