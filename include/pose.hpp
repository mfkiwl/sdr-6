#ifndef POSE_HPP
#define POSE_HPP
#pragma once

#include <fstream>

#include <Eigen/Dense>

/**
  * @brief Declarations for functionality relating to processing pose (position, orientation) related items
  */

namespace sdr {

    using position_t = Eigen::Matrix<double, 1, 3> ; // 1 * 3 matrix (xyz position)
    using rotation_m_t = Eigen::Matrix<double, 3, 3> ; // 3 * 3 matrix (rot. matrix)
    using quaternion_t = Eigen::Quaternion<double, Eigen::AutoAlign> ; // 4 * 1 matrix

    class Pose {
    /**
      * @brief Pose (class) - class to strictly to manage pose information (ie. distance and orientation changes)
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
              * @param const double - new distance travelled along x axis
              * @param const double - new distance travelled along y axis
              * @param const double - new distance travelled along z axis
              * @throws sdr::DetailedException - thrown in case of invalid angle ranges
              */
            void update_position(const double, const double, const double) noexcept ;

            /**
              * @brief update_orientation - calculates and applies local orientation changes in a global map
              * @param const double - yaw angle in radians (rotation around z axis)
              * @param const double - pitch angle in radians (rotation around y axis)
              * @param const double - roll angle in radians (rotation around x axis)
              * @throws sdr::DetailedException - thrown in case of invalid angle ranges (angle < -1 || angle > 1)
              */
            void update_orientation(const double, const double, const double) noexcept(false) ;

            // below are defaulted and deleted methods
            Pose(const Pose&) noexcept = default ; // copy constructor
            Pose& operator=(const Pose&) noexcept = default ; // copy assignment operator
            Pose(Pose&&) noexcept = default ; // move constructor
            Pose& operator=(Pose&&) noexcept = default ; // move assignment operator
            ~Pose() noexcept = default ;

            friend ::std::ostream& operator<<(::std::ostream&, const Pose&) noexcept ;

    } ;

    /**
      * @brief output stream operator (<<) (overload) - function to print Pose object
      * @param std::ostream& - reference to out stream object to write text to
      * @param const Pose& - const reference to Pose object
      * @return std::ostream& - reference to updated out stream object
      */
    ::std::ostream& operator<<(::std::ostream&, const Pose&) noexcept ;

    /**
      * @brief velocities_to_deltas - uses time information to return distances / angles achieved on / around each axis based on given velocities
      * @param const std::vector<double>& - const reference to list of velocities recorded on / around a given axis
      * @return std::vector<double> - list of distances / angles calculated around a given axis
      */
    std::vector<double> velocities_to_deltas(const ::std::vector<double>&, const double) noexcept ;

} ; // namespace sdr

#endif // POSE_HPP
