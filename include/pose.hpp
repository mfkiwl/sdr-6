#ifndef POSE_HPP
#define POSE_HPP
#pragma once

#include <Eigen/Dense>

/**
  * @brief Declarations for functionality relating to processing pose (position, orientation) related items
  */

namespace sdr {

    using position_t = Eigen::Matrix<float, 1, 3> ; // 1 * 3 matrix (xyz position)
    using rotation_m_t = Eigen::Matrix<float, 3, 3> ; // 3 * 3 matrix (rot. matrix)
    using quaternion_t = Eigen::Quaternion<float, Eigen::AutoAlign> ; // 4 * 1 matrix

    class Pose {
            /**
              * @brief Pose (class) - class to analyse validity and track pose related information
              */
       private:
            position_t _position ;

            rotation_m_t _orientation ;

        public:
            /**
              * @brief Pose (constructor) - empty initialiser
              */
            Pose() noexcept ;

            /**
              * @brief Pose (constructor) - assesses and sets assigned values
              * @param const sdr::position_t - initial specified position
              * @param const sdr::quaternion_t - initial specified orientation (quaternions - rotation matrix handled internally due to easier maths but default orientation is quaternions so can't expect any different)
              */
            Pose(const position_t&, const quaternion_t&) noexcept(false) ;

            /**
              * @brief position - getter method which returns translation
              * @return sdr::position_t - xyz matrix correlating to current position
              */
            position_t position() const noexcept ;

            /**
              * @brief orientation - getter method which returns orientation in quaternion format (not really a getter but oh well)
              * @return sdr::quaternion_t - quaternion *converted* from rotation matrix property
              */
            quaternion_t orientation() const noexcept ;

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

            // below are defaulted and deleted methods
            Pose(const Pose&) noexcept = default ; // copy constructor
            Pose& operator=(const Pose&) noexcept = default ; // copy assignment operator
            Pose(Pose&&) noexcept = default ; // move constructor
            Pose& operator=(Pose&&) noexcept = default ; // move assignment operator
            ~Pose() noexcept = default ;

    } ;

}

#endif // POSE_HPP
