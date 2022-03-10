#include <iostream>
#include <fstream>
#include <utility>

#include <argp.h>

#include <kalman/UnscentedKalmanFilter.hpp>

#include "preprocessing.hpp"
#include "detailed_exception.hpp"
#include "pose.hpp"
#include "unscented_kalman_support.hpp"

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
        {"initial_pose", 'p', "YAML_FILE", 0, "Reads an initial YAML file containing initial position & orientation in a world"},
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

    sdr::Pose pose ; // empty 0 default initialisation
    if(arguments.initial_pose_file)
    {
        pose = sdr::extract_initial_pose(std::string(arguments.initial_pose_file)) ; // actually extract information from given file
    }
    std::cout << "Starting:\n\t" << pose << std::endl ;

    /* Set up Kalman filter vectors */
    sdr::KalmanState state ;
    state.setZero() ;
    sdr::KalmanControl control ;
    Kalman::UnscentedKalmanFilter<sdr::KalmanState> ukf(1) ; // convariance
    ukf.init(state) ;
    sdr::SystemModel sys_model ;
    sdr::MeasurementModel measurement_model ;

    while(true)
    {
        float distance_x_askd = 0.f ;
        float distance_y_askd = 0.f ;
        float distance_z_askd = 0.f ;
        float delta_yaw_askd = 0.f ;
        float delta_pitch_askd = 0.f ;
        float delta_roll_askd = 0.f ;

        float distance_x_sensd = 0.f ;
        float distance_y_sensd = 0.f ;
        float distance_z_sensd = 0.f ;
        float delta_yaw_sensd = 0.f ;
        float delta_pitch_sensd = 0.f ;
        float delta_roll_sensd = 0.f ;

        input >> distance_x_askd >> distance_y_askd >> distance_z_askd >> delta_yaw_askd >> delta_pitch_askd >> delta_roll_askd ;
        input >> distance_x_sensd >> distance_y_sensd >> distance_z_sensd >> delta_yaw_sensd >> delta_pitch_sensd >> delta_roll_sensd ;

        if(input.eof())
        {
            break ;
        }

        /* Set control vector values - values which were actually requested */
        control.delta_x() = distance_x_askd ;
        control.delta_y() = distance_y_askd ;
        control.delta_z() = distance_z_askd ;
        control.delta_yaw() = delta_yaw_askd ;
        control.delta_pitch() = delta_pitch_askd ;
        control.delta_roll() = delta_roll_askd ;

        /* Based on requested values, simulate what the values would ideally be  */
        state = sys_model.f(state, control) ;
        std::cout << "state guessed " << state << std::endl ;

        /* Now make a different object which stores what was ACTUALLY recorded */

//        state.delta_x() = distance_x_sensd ;
//        state.delta_y() = distance_y_sensd ;
//        state.delta_z() = distance_z_sensd ;
//        state.delta_yaw() = delta_yaw_sensd ;
//        state.delta_pitch() = delta_pitch_sensd ;
//        state.delta_roll() = delta_roll_sensd ;
//
//        auto ukf_pred = ukf.predict(sys_model, control) ; // predict values of next time-step
        sdr::Measurement measurement = measurement_model.h(state) ;
        sdr::KalmanState ukf_pred = ukf.update(measurement_model, measurement) ; // Update UKF

//        std::cout << "ukf guessed x: " << ukf_pred.x() << std::endl ;
//        std::cout << "ukf guessed y: " << ukf_pred.y() << std::endl ;
//        std::cout << "ukf guessed z: " << ukf_pred.z() << std::endl ;
//        std::cout << "ukf guessed yaw: " << ukf_pred.yaw() << std::endl ;
//        std::cout << "ukf guessed pitch: " << ukf_pred.pitch() << std::endl ;
//        std::cout << "ukf guessed roll: " << ukf_pred.roll() << std::endl ;
//
//        pose.update_position(ukf_pred.x(), ukf_pred.y(), ukf_pred.z()) ;
//        pose.update_orientation(delta_yaw, delta_pitch, delta_roll) ;
//        std::cout << pose << std::endl ;
    }

//    std::cout << "Final:\n\t" << pose << std::endl ;
    //
    return 0 ;
}
