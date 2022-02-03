#include <iostream>
#include <argp.h>

#include "preprocessing.hpp"
#include "detailed_exception.hpp"
#include "pose.hpp"

/**
  * @brief Main source file managing sdr system
  * Note: this source file acts *less* as real-time way to track positioning, rather simply shows how it's done. The main body of the program (ie. everything besides input initialisation) would be kept the same. Hence, the ROS implementation of this repository acts as a wrapper
  */

#pragma GCC diagnostic ignored "-Wmissing-field-initializers" // Below is some argp stuff. I'm ignoring some of the 'errors'
#pragma GCC diagnostic push

    static char args_doc[] = "LOG_PATH" ; // description of non-option specified command line arguments
    static char doc[] = "sdr -- a simple dead reckoning application" ; // general program documentation
    const char* argp_program_bug_address = "salih.msa@outlook.com" ;
    static struct argp_option options[] = {
        {"pose", 'p', "YAML_FILE", 0, "Reads an initial YAML file containing initial position & orientation in a world"},
        {0}
    } ;
    struct arguments {
        /** @brief struct arguments - this structure is used to communicate with parse_opt (for it to store the values it parses within it) **/
        char* args[2] ;  /* args for params */
        char* initial_pose_file ;
    } ;


    /** @brief parse_opt - deals with given arguments based on given argumentsK
      * @param int - int correlating to char storing argument key
      * @param char* - argument string associated with argument key
      * @param struct argp_state* - pointer to argp_state struct storing information about the state of the option parsing
      * @return error_t - number storing 0 upon successfully parsed values, non-zero exit code otherwise **/
    static error_t parse_opt(int key, char *arg, struct argp_state* state)
    {
        struct arguments* arguments = (struct arguments*)state->input;

        switch (key)
        {
            case 'p':
                arguments->initial_pose_file = arg ;
                break ;
            case ARGP_KEY_ARG:
                if(state->arg_num >= 2)
                {
                    argp_usage(state);
                }
                arguments->args[state->arg_num] = arg;
                break;
            case ARGP_KEY_END:
                if (state->arg_num < 1)
                {
                    argp_usage(state);
                }
                break;
            default:
                return ARGP_ERR_UNKNOWN;
        }
        return 0 ;
    }

#pragma GCC diagnostic pop // end of argp, so end of repressing weird messages

int main(int argc, char** argv)
{
    /* Initialisation */
    struct arguments arguments;
    arguments.initial_pose_file = nullptr ;
    static struct argp argp = { // argp - The ARGP structure itself
        options, // options
        parse_opt, // callback function to process args
        args_doc, // names of parameters
        doc // documentation containing general program description
    } ;
    argp_parse(&argp, argc, argv, 0, 0, &arguments); // override default arguments if provided

    sdr::Pose pose ;

    if(arguments.initial_pose_file)
    {
        pose = sdr::extract_initial_pose(std::string(arguments.initial_pose_file)) ; // actually extract information from given file
    }

    for(int i = 0 ; i < 2 ; ++i)
    {
        const float distance_x = 1.f ;
        const float distance_y = 0.f ;
        const float distance_z = 0.f ;

        const float angle_x = 0.785398 ;
        const float angle_y = 0.f ;
        const float angle_z = 0.f ;

        pose.update_position(distance_x, distance_y, distance_z) ;
        pose.update_orientation(angle_x, angle_y, angle_z) ;
        std::cout << "Global position:\n\t" << pose.position() << std::endl ;
        std::cout << "Global orientation:\n\t" << pose.orientation() << std::endl ;
    }

    std::cout << "Final position: " << pose.position() << std::endl ;
    //
    return 0 ;
}
