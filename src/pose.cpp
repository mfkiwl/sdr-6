#include <cmath>
#include <string>

#include <Eigen/Geometry>

#include "detailed_exception.hpp"
#include "pose.hpp"

/**
  * @brief Definitions for functionality relating to processing pose (position, orientation) related items
  */

sdr::Pose::Pose() noexcept
{
    this->_position = sdr::position_t::Zero() ;
    this->_orientation = sdr::rotation_m_t{
        {1,0,0},
        {0,1,0},
        {0,0,1}
    } ;
}

sdr::Pose::Pose(const sdr::position_t& initial_position, const sdr::quaternion_t& initial_orientation) noexcept(false)
{
    if(initial_position.rows() != 1 || initial_position.cols() != 3)
    {
        const std::string msg{"Translation matrix dimensions provided are invalid. Required: 1*3. Provided: " + std::to_string(initial_position.rows()) + "*" + std::to_string(initial_position.cols())} ;
        throw sdr::DetailedException(__func__, __LINE__, msg) ;
    }
    this->_position = initial_position ;

    this->_orientation = initial_orientation.normalized().toRotationMatrix() ; // checks will occur before this point to ensure it is in valid quaternion format
}

sdr::position_t sdr::Pose::position() const noexcept
{
    return this->_position ;
}

sdr::quaternion_t sdr::Pose::orientation() const noexcept
{
    return sdr::quaternion_t{this->_orientation} ;
}

void sdr::Pose::update_position(const float delta_x, const float delta_y, const float delta_z) noexcept
{
        // caertesian 3d coordinates
    sdr::position_t delta_translation{
        {delta_x,delta_y,delta_z}
    } ; // local changes
    delta_translation *= this->_orientation ; // multiply by global orientation to determine its global significance
    this->_position += delta_translation ;
}

void sdr::Pose::update_orientation(const float yaw, const float pitch, const float roll) noexcept(false)
{
    auto valid_angle = [](const float rad_angle) -> bool {
        return (rad_angle < -2.f || rad_angle > 2.f ? false : true) ;
    } ;

    if(!(valid_angle(yaw) && valid_angle(pitch) && valid_angle(yaw)))
    {
        const std::string msg{ "Ill ranging values (radians have a max radian degree of 2 and a min radian degree of -2). Values provided: " + std::to_string(yaw) + ", " + std::to_string(pitch) + ", " + std::to_string(roll) } ;
        throw sdr::DetailedException(__func__, __LINE__, msg) ;
    }

    const sdr::rotation_m_t rot_matrix_yaw{ // rotation around z axis (left/right)
                                                {std::cos(yaw),std::asin(yaw),0.f},
                                                {std::sin(yaw),std::cos(yaw),0.f},
                                                {0.f,0.f,1.f}
                                            } ;

    const sdr::rotation_m_t rot_matrix_pitch{ // rotation around y axis (up/down)
                                                 {std::cos(pitch),0.f,std::sin(pitch)},
                                                 {0.f,1.f,0.f},
                                                 {std::asin(pitch),0.f,std::cos(pitch)}
                                             } ;

    const sdr::rotation_m_t rot_matrix_roll{ // rotation around x axis (tilting) (won't be applicable to cars etc.)
                                               {1.f,0.f,0.f},
                                               {0.f,std::cos(roll),std::asin(roll)},
                                               {0.f,std::sin(roll),std::cos(roll)}
                                           } ;

    const sdr::rotation_m_t delta_orientation = rot_matrix_yaw * rot_matrix_pitch * rot_matrix_roll ;
    this->_orientation *= delta_orientation ;
}
