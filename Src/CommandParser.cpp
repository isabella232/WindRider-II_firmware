//! Command Parser implementation file.
/**
 * @file      CommandParser.cpp
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 */

#include "CommandParser.h"
#include "HardwareDriver.h"

//! Namespace CommandParser
namespace CommandParser{

    // Command table. Please keep alphabetical order.
    // TODO: Figure out const correctness
    const std::array<Cmd, 8> command_table = {
        Cmd("current", " \t-- print suction motor current:\n\r\t\t\t\t1) current", HardwareDriver::suction_current),
        Cmd("led", " \t\t-- set led current:\n\r\t\t\t\t1) led [#channel 0-3] [current 0-1500] -- set led current in mA", HardwareDriver::led),
        Cmd("move", "\t\t-- asynchronously set motor speed:\n\r\t\t\t\t1) move [left speed] [right speed] -- speed to be loaded into the motor drivers.", HardwareDriver::move),
        Cmd("servo", " \t\t-- set servo angle:\n\r\t\t\t\t1) servo [#channel 0-1] [0,180]", HardwareDriver::servo_angle),
        Cmd("solenoid", " \t-- manage impactor solenoids:\n\r\t\t\t\t1) solenoid [#channel] [off]\n\r\t\t\t\t2) solenoid [#channel] [on_time_ms] [off_time_ms]", HardwareDriver::solenoid),
        Cmd("straight", "\t-- set and synchronize motor speed:\n\r\t\t\t\t1) straight [speed] -- speed to be loaded into both right and left motor driver synchronously.", HardwareDriver::sync_straight),
        Cmd("suction", " \t-- manage suction:\n\r\t\t\t\t1) suction [on/off] -- enable disable suction\n\r\t\t\t\t2) suction [0-100] -- set suction power", HardwareDriver::suction_power),
        Cmd("uart", "\t\t-- uart forwarding:\n\r\t\t\t\t1) uart [forward] [\"Command\"] -- forward a string via uart\n\r\t\t\t\t2) uart [reply] [on/off] -- print incoming feedback", HardwareDriver::uart)
    };

    const std::string execute(const std::string &command){

        // Check if help requested.
        if(command[0] == '?'){
            std::string help_str = help;

            // Return a string with all the tokens and help strings.
            std::for_each(command_table.begin(),
                          command_table.end(),
                          [&help_str](const Cmd &cmd_table_entry){

                              help_str += cmd_table_entry.token;
                              help_str += cmd_table_entry.help;
                              help_str += "\n\r";
                          });
            help_str + "\r";

            return help_str;
        }

        // Find the first space to seperate a token.
        auto token_separator = command.find_first_of(' ');

        // If there is no space, assume command has no arguments.
        if(token_separator == std::string::npos)
            token_separator = command.size()-1;

        // Lookup the given token in the command table.
        // In case found, returns a poiter to the corresponding command container.
        const auto executable = std::find_if(command_table.begin(), 
                                             command_table.end(), 
                                             [&command, &token_separator](const Cmd &cmd_table_entry){

                                    return cmd_table_entry.token == command.substr(0, token_separator);
                                });

        // Check if given token exists in the command table.
        if(executable == command_table.end())
            return unknown_command;

        // Parse arguments
        // Instantiate an empty vector of args.
        std::vector<std::string> args;

        // If command contains any arguments
        if(token_separator != command.size()-1)
            parse_args_recursive(command.substr(token_separator + 1), args);

        // Execute the corresponding command from the table.
        executable->execute(args);

        // Return status of the executed command.
        return HardwareDriver::get_status();
    }

    void parse_args_recursive(const std::string args_string, std::vector<std::string> &args_vector){

        const auto separator = args_string.find_first_of(' ');

        // No more args found
        if(separator == std::string::npos)
            return
        
        args_vector.push_back(args_string.substr(0, separator));

        parse_args_recursive(args_string.substr(separator + 1), args_vector);

    }

}
