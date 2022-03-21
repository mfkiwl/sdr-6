#include <iostream>
#include <fstream>
#include <utility>
#include <cstdlib>
#include <tuple>
#include <vector>
#include <array>

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

    static char args_doc[] = "LOG_PATH NUM_SOURCES" ; // description of non-option specified command line arguments
    static char doc[] = "sdr -- a simple dead reckoning application" ; // general program documentation
    const char* argp_program_bug_address = "salih.msa@outlook.com" ;
    static struct argp_option options[] = {
        {"initial_pose", 'p', "YAML_FILE", 0, "Reads an initial YAML file containing initial position & orientation in a world"},
        {0}
    } ;
    struct arguments {
        /** @brief struct arguments - this structure is used to communicate with parse_opt (for it to store the values it parses within it) **/
        char* args[3] ;  /* args for params */
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
                if(state->arg_num >= 3)
                {
                    argp_usage(state);
                }
                arguments->args[state->arg_num] = arg;
                break;
            case ARGP_KEY_END:
                if (state->arg_num < 2)
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

namespace sdr {

    /**
      * @brief read_log_entry - reads entry to log (twist message and time spent doing said velocity) from text file. not really useful for general dead reckoning purposes, hence in this example usage file
      * @param std::istream& - mutable reference to input stream object connected log file
      * @param const std::size_t - number of different inputs / sensor readings for each given entry (ie. 2 sensors reporting twist msgs for each entry)
      */
    ::std::tuple<\
               ::std::vector<double>, ::std::vector<double>, ::std::vector<double>, \
               ::std::vector<double>, ::std::vector<double>, ::std::vector<double>, \
               double\
              > read_log_entry(::std::istream& input, const ::std::size_t number_of_sources) noexcept(false)
    {
        /* Read in velocity values along each axis as well as time spent in said velocities */
        static std::vector<double> linear_vels_x(number_of_sources), linear_vels_y(number_of_sources), linear_vels_z(number_of_sources) ;
        static std::vector<double> angular_vels_x(number_of_sources), angular_vels_y(number_of_sources), angular_vels_z(number_of_sources) ;
        double time = 0.f ;

        for(std::size_t i = 0 ; i < number_of_sources ; ++i)
        {
            input >> linear_vels_x[i] >> linear_vels_y[i] >> linear_vels_z[i] ;
            input >> angular_vels_x[i] >> angular_vels_y[i] >> angular_vels_z[i] ;
        }
        input >> time ;

        if(input.eof()) // if the full read was not successful
        {
            const std::string msg = "Reading entry failed due to incompleteness" ;
            throw sdr::DetailedException(__func__, static_cast<unsigned int>(__LINE__), msg) ;
        }

        return {
            linear_vels_x, linear_vels_y, linear_vels_z,
            angular_vels_x, angular_vels_y, angular_vels_z,
            time
        } ;
    }

} ; // namespace sdr

int main(int argc, char** argv)
{
    /* Initialisation */
    struct arguments arguments ;
    arguments.initial_pose_file = nullptr ;
    static struct argp argp = { // argp - The ARGP structure itself
        options, // options
        parse_opt, // callback function to process args
        args_doc, // names of parameters
        doc // documentation containing general program description
    } ;
    argp_parse(&argp, argc, argv, 0, 0, &arguments); // override default arguments if provided

    if(!sdr::is_meta_file(std::string(arguments.args[0])))
    {
        const std::string msg = "'" + std::string(arguments.args[0]) + "' is not a valid file or a symlink to a valid file" ;
        throw sdr::DetailedException(__func__, static_cast<unsigned int>(__LINE__), msg) ;
    }
    std::fstream input(std::string{arguments.args[0]}) ; // initialise text stream object of movement logs etc.

    const int number_of_sources = std::atoi(arguments.args[1]) ;
    if(number_of_sources < 1)
    {
        const std::string msg = "'" + std::to_string(number_of_sources) + "' provided as the number of sources - should be a positive non-zero integer" ;
        throw sdr::DetailedException(__func__, static_cast<unsigned int>(__LINE__), msg) ;
    }

    sdr::Pose pose ; // empty 0 center default initialisation
    if(arguments.initial_pose_file)
    {
        pose = sdr::extract_initial_pose(std::string(arguments.initial_pose_file)) ; // actually extract information from given file
    }
    std::cout << "Starting:\n\t" << pose << std::endl ;

    /* Main functionality */
    auto at_end = [&]() -> bool
    {
        input.get() ;
        input.get() ;
        if(input.eof())
            return true ;
        input.unget() ;
        input.unget() ;
        return false ;
    } ;

    while(!at_end())
    {
        /* Read in velocity values along each axis as well as time spent in said velocities */
        const auto [linear_vels_x, linear_vels_y, linear_vels_z, angular_vels_x, angular_vels_y, angular_vels_z, time] = sdr::read_log_entry(input, static_cast<std::size_t>(number_of_sources)) ;

        /* Process preliminary input */
        const auto deltas_x = sdr::velocities_to_deltas(linear_vels_x, time) ;
        const auto deltas_y = sdr::velocities_to_deltas(linear_vels_y, time) ;
        const auto deltas_z = sdr::velocities_to_deltas(linear_vels_z, time) ;
        const auto rolls = sdr::velocities_to_deltas(angular_vels_x, time) ;
        const auto pitches = sdr::velocities_to_deltas(angular_vels_y, time) ;
        const auto yaws = sdr::velocities_to_deltas(angular_vels_z, time) ;

        /* Process final output */
        pose.update_position(deltas_x[0], deltas_y[0], deltas_z[0]) ;
        pose.update_orientation(rolls[0], pitches[0], yaws[0]) ;
        std::cout << pose << std::endl ;
    }

    std::cout << "Final:\n\t" << pose << std::endl ;
    //
    return 0 ;
}
