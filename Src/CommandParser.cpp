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
    const std::array<Cmd, 6> command_table = {
        Cmd("current", " \t-- print suction motor current:\n\r\t\t\t\t1) current", HardwareDriver::suction_current),
        Cmd("led", " \t\t-- set led current:\n\r\t\t\t\t1) led [#channel 0-3] [current 0-1500] -- set led current in mA", HardwareDriver::led),
        Cmd("servo", " \t\t-- set servo angle:\n\r\t\t\t\t1) servo [#channel 0-1] [0,180]", HardwareDriver::servo_angle),
        Cmd("solenoid", " \t-- manage impactor solenoids:\n\r\t\t\t\t1) solenoid [#channel] [off]\n\r\t\t\t\t2) solenoid [#channel] [on_time_ms] [off_time_ms]", HardwareDriver::solenoid),
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

        // Parse tokens into a vector
        std::vector<std::string> tokens = {};

        parse_tokens(command, tokens);

        // Lookup the first positioned token in the command table.
        // In case found, returns a poiter to the corresponding command container.
        const auto executable = std::find_if(command_table.begin(), 
                                             command_table.end(), 
                                             [&tokens](const Cmd &cmd_table_entry){

                                    return cmd_table_entry.token == tokens.front();
                                });

        tokens.erase(tokens.begin());

        // Check if given token exists in the command table.
        if(executable == command_table.end())
            return unknown_command;

        // Execute the corresponding command from the table.
        executable->execute(tokens);
  
        // Return status of the executed command.
        return HardwareDriver::get_status();
    }

    void parse_tokens(const std::string& args_string, std::vector<std::string> &args_vector, const char delimiter){

        std::string::size_type last_pos = 0;

        while(last_pos < args_string.size()){

            auto separator = args_string.find_first_of(delimiter, last_pos);

            if(separator == std::string::npos)
                separator = args_string.size()-1;

            args_vector.push_back(args_string.substr(last_pos, separator-last_pos));
            
            last_pos = separator + 1;
        }
    }
}
