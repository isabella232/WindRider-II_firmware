//! CommandParser declaration file.
/**
 * @file      CommandParser.h
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 *
 */

#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include <string>
#include <algorithm>
#include <functional>

namespace CommandParser{

    //! Command container class.
    /**
     *  @brief This container is used to place commands into the command lookup table.
     */
    class Cmd {

        public:

        //! Default Cmd constructor.
        /**
         * @param token Command token used to call the specified command.
         * @param help  Help string.
         * @param cmd_handler Function to be executed when command with the token is called.
         */ 
        Cmd(std::string token, std::string help, std::function<void(std::string)> cmd_handler):
            token(token),help(help),cmd_handler(cmd_handler){};

        //! execute method.
        /**
         * @brief Public method to call the command handler.
         * @param arg Argument string passed to the command handler. 
         */ 
        void execute(std::string arg){

            cmd_handler(arg);
        };
        
        const std::string token; // Command token.
        const std::string help; // Help string.

        private:

        // Command Handler function pointer.
        const std::function<void(std::string)> cmd_handler;
    };

    //! Execute method.
    /**
     * @brief This method processes a raw received command.
     *          1. Seperate token and space seperated args from the raw command.
     *          2. Lookup the token in the command table.
     *          3. In case the token exists in the table, 
     *             execute the corresponding commnad handler passin the space seperated argument.
     *          4. Return a status string.
     * @param arg Raw command to execute as received from host.
     * @return    Status string.
     */ 
    const std::string execute(const std::string &command);

    // Status strings.
    const std::string unknown_command = "unknown command\n\r";
    const std::string invalid_command = "invalid command enter '?' for help\n\r";
    const std::string help = "CCNY Robotics Lab Welcomes You, human..\n\r";

};

#endif //COMMANDPARSER_H
