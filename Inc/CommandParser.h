//! Command Parser declaration file.
/**
 * @file      CommandParser.h
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 *
 * This header provides the definitions related to register implementation.
 */

#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include <string>
#include <algorithm>
#include <functional>

namespace CommandParser {

    //! Command container class.
    /**
     *  @brief This container is used to place commands into the command table.
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

        //! Cmd constructor.
        /**
         * @brief Empty help string constructor.
         */ 
        Cmd(std::string token, std::function<void(std::string)> cmd_handler):
            token(token),help(),cmd_handler(cmd_handler){};

        //! execute method.
        /**
         * @brief Public method to call the command handler.
         * @param arg Argument string passed to the command handler. 
         */ 
        void execute(std::string arg){

            cmd_handler(arg);
        };
        
        const std::string token;
        const std::string help;

        private:

        // Command Handler function pointer.
        const std::function<void(std::string)> cmd_handler;
    };

    //! execute method.
    /**
     * @brief Public method to call the command handler.
     * @param arg Argument string passed to the command handler. 
     */ 
    const std::string execute(const std::string &command);

    // Responses
    const std::string unknown_command = "unknown command\n\r";
    const std::string invalid_command = "invalid command enter '?' for help\n\r";
    const std::string help = "CCNY Robotics Lab Welcomes You, human..\n\r";

};

#endif //COMMANDPARSER_H
