#ifndef POSE_HPP
#define POSE_HPP
#pragma once

#include <Eigen/Dense>

/**
  * @brief Declarations for functionality relating to processing pose (position, orientation) related items
  */

namespace sdr {

    using position_t = Eigen::Matrix<float, 1, 3> ; // 1 * 3 matrix (xyz position)
    using orientation_t = Eigen::Matrix<float, 3, 3> ; // 3 * 3 matrix (rot. matrix)

    class Pose {
            /**
              * @brief Pose (class) - class to analyse validity and track pose related information
              */
       private:
            position_t _position ;

            orientation_t _orientation ;

        public:
            /**
              * @brief Pose (constructor) - empty initialiser
              */
            Pose() noexcept ;

            /**
              * @brief Pose (constructor) - assesses and sets assigned values
              * @param const sdr::position_t - initial specified position
              * @param const sdr::orientation_t - initial specified orientation (rotation matrix format)
              */
            Pose(const position_t&, const orientation_t&) noexcept(false) ;

            /**
              * @brief position - getter method which returns translation
              * @return sdr::position_t - xyz matrix correlating to current position
              */
            position_t position() const noexcept ;

            /**
              * @brief orientation - getter method which returns orientation in rotation matrix format
              * @return sdr::orientation_t - rotation matrix correlating to current orientation
              */
            orientation_t orientation() const noexcept ;

            /**
              * @brief update_position - method which calculates local changes in translation in a global map
              * @param const float - new distance travelled along caertesian x axis
              * @param const float - new distance travelled along caertesian y axis
              * @param const float - new distance travelled along caertesian z axis
              * @throws sdr::DetailedException - thrown in case of invalid angle ranges
              */
            void update_position(const float, const float, const float) noexcept ;

            /**
              * @brief update_orientation - calculates and applies local orientation changes in a global map
              * @param const float - yaw angle in radians (rotation around caertesian z axis)
              * @param const float - pitch angle in radians (rotation around caertesian y axis)
              * @param const float - roll angle in radians (rotation around caertesian x axis)
              * @throws sdr::DetailedException - thrown in case of invalid angle ranges (angle < -1 || angle > 1)
              */
            void update_orientation(const float, const float, const float) noexcept(false) ;

            /**
              * @brief calculate_delta_position - static method which calculates local changes in translation in a global map
              * @param const float - new distance travelled along caertesian x axis
              * @param const float - new distance travelled along caertesian y axis
              * @param const float - new distance travelled along caertesian z axis
              * @param const sdr::orientation_t& - const reference to global orientation
              * @throws sdr::DetailedException - thrown in case of calculated values involving Nans
              * @return sdr::position_t - deduced change in translation
              */
            static position_t calculate_delta_position(const float, const float, const float, const orientation_t&) noexcept ;

            /**
              * @brief calculate_delta_orientation - calculate local change based on heading and gyro data
              * @param const float - yaw angle (rotation around caertesian z axis)
              * @param const float - pitch angle (rotation around caertesian y axis)
              * @param const float - roll angle (rotation around caertesian x axis)
              * @throws sdr::DetailedException - thrown in case of calculated values involving Nans
              * @return sdr::orientation_t - deduced change in orientation
              */
            static orientation_t calculate_delta_orientation(const float, const float, const float) noexcept(false) ;

            // below are defaulted and deleted methods
            Pose(const Pose&) noexcept = default ; // copy constructor
            Pose& operator=(const Pose&) noexcept = default ; // copy assignment operator
            Pose(Pose&&) noexcept = default ; // move constructor
            Pose& operator=(Pose&&) noexcept = default ; // move assignment operator
            ~Pose() noexcept = default ;

    } ;

}

#endif // POSE_HPP
