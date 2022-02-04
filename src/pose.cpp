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
    this->_orientation = sdr::orientation_t{
        {1,0,0},
        {0,1,0},
        {0,0,1}
    } ;
}

sdr::Pose::Pose(const sdr::position_t& initial_position, const sdr::orientation_t& initial_orientation) noexcept(false)
{
    this->_position = initial_position ; // can't really check translation, you can theoretically be valid anywhere
    this->_orientation = initial_orientation ;
}

sdr::position_t sdr::Pose::position() const noexcept
{
    return this->_position ;
}

sdr::orientation_t sdr::Pose::orientation() const noexcept
{
    return this->_orientation ;
}

void sdr::Pose::update_position(const float delta_x, const float delta_y, const float delta_z) noexcept
{
    const auto delta_postion = sdr::Pose::calculate_delta_position(delta_x, delta_y, delta_z, this->_orientation) ;
    this->_position += delta_postion ;
}

void sdr::Pose::update_orientation(const float delta_yaw, const float delta_pitch, const float delta_roll) noexcept(false)
{
    const auto delta_orientation = sdr::Pose::calculate_delta_orientation(delta_yaw, delta_pitch, delta_roll) ;
    this->_orientation *= delta_orientation ;
}

sdr::position_t sdr::Pose::calculate_delta_position(const float delta_x, const float delta_y, const float delta_z, const orientation_t& global_orientation) noexcept
{
    // caertesian 3d coordinates
    sdr::position_t delta_translation{
        {delta_x,delta_y,delta_z}
    } ; // local changes
    delta_translation *= global_orientation ; // multiply by global orientation to determine its global significance
    return delta_translation ;
}

sdr::orientation_t sdr::Pose::calculate_delta_orientation(const float delta_yaw, const float delta_pitch, const float delta_roll) noexcept(false)
{
    auto valid_angle = [](const float delta_rad_angle) -> bool {
        return (delta_rad_angle < -2.f || delta_rad_angle > 2.f ? false : true) ;
    } ;

    if(!(valid_angle(delta_yaw) && valid_angle(delta_pitch) && valid_angle(delta_yaw)))
    {
        const std::string msg{ "Ill ranging values (radians have a max radian degree of 2 and a min radian degree of -2). Values provided: " + std::to_string(delta_yaw) + ", " + std::to_string(delta_pitch) + ", " + std::to_string(delta_roll) } ;
        throw sdr::DetailedException(__func__, __LINE__, msg) ;
    }


    const sdr::orientation_t rot_matrix_yaw{ // rotation around z axis (left/right)
                                                {std::cos(delta_yaw),std::asin(delta_yaw),0.f},
                                                {std::sin(delta_yaw),std::cos(delta_yaw),0.f},
                                                {0.f,0.f,1.f}
                                            } ;

    const sdr::orientation_t rot_matrix_pitch{ // rotation around y axis (up/down)
                                                 {std::cos(delta_pitch),0.f,std::sin(delta_pitch)},
                                                 {0.f,1.f,0.f},
                                                 {std::asin(delta_pitch),0.f,std::cos(delta_pitch)}
                                             } ;

    const sdr::orientation_t rot_matrix_roll{ // rotation around x axis (tilting) (won't be applicable to cars etc.)
                                               {1.f,0.f,0.f},
                                               {0.f,std::cos(delta_roll),std::asin(delta_roll)},
                                               {0.f,std::sin(delta_roll),std::cos(delta_roll)}
                                           } ;

    const sdr::orientation_t delta_orientation = rot_matrix_yaw * rot_matrix_pitch * rot_matrix_roll ;
    return delta_orientation ;
}
