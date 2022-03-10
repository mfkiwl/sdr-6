#ifndef UNSCENTED_KALMAN_SUPPORT_HPP
#define UNSCENTED_KALMAN_SUPPORT_HPP
#pragma once

#include <cstddef>

#include <kalman/LinearizedMeasurementModel.hpp>
#include <kalman/LinearizedSystemModel.hpp>

#include "pose.hpp"

/**
  * @brief File contains declarations of functionality to support and simplify use of external Kalman filter library, specifically the unscented kalman filter
  */

namespace sdr {

    class KalmanState : public ::Kalman::Vector<float, 6> {
        public:
            KALMAN_VECTOR(KalmanState, float, 6) // weird macro, hence no semicolon needed

            constexpr static ::std::size_t X = 0 ;

            inline void x(const float val) noexcept
            {
                (*this)[X] = val ;
            }

            inline float& x() noexcept
            {
                return (*this)[X] ;
            }

            inline float x() const noexcept
            {
                return (*this)[X] ;
            }

            constexpr static ::std::size_t Y = 1 ;

            inline void y(const float val) noexcept
            {
                (*this)[Y] = val ;
            }

            inline float& y() noexcept
            {
                return (*this)[Y] ;
            }

            inline float y() const noexcept
            {
                return (*this)[Y] ;
            }

            constexpr static ::std::size_t Z = 2 ;

            inline void z(const float val) noexcept
            {
                (*this)[Z] = val ;
            }

            inline float& z() noexcept
            {
                return (*this)[Z] ;
            }

            inline float z() const noexcept
            {
                return (*this)[Z] ;
            }

            constexpr static ::std::size_t YAW = 3 ;

            inline void yaw(const float val) noexcept
            {
                (*this)[YAW] = val ;
            }

            inline float& yaw() noexcept
            {
                return (*this)[YAW] ;
            }

            inline ::std::size_t yaw() const noexcept
            {
                return (*this)[YAW] ;
            }

            constexpr static ::std::size_t PITCH = 4 ;

            inline void pitch(const float val) noexcept
            {
                (*this)[PITCH] = val ;
            }

            inline float& pitch() noexcept
            {
                return (*this)[PITCH] ;
            }

            inline float pitch() const noexcept
            {
                return (*this)[PITCH] ;
            }

            constexpr static ::std::size_t ROLL = 5 ;

            inline void roll(const float val) noexcept
            {
                (*this)[ROLL] = val ;
            }

            inline float& roll() noexcept
            {
                return (*this)[ROLL] ;
            }

            inline float roll() const noexcept
            {
                return (*this)[ROLL] ;
            }

    } ;

    ::std::ostream& operator<<(::std::ostream& os, const KalmanState& ks)
    {
        os << "x: " << ks.x() << ", y: " << ks.y() << ", z: " << ks.z() << ", yaw: " << ks.yaw() << ", pitch: " << ks.pitch() << ", roll: " << ks.roll() << '\n' ;
        return os ;
    }

    class KalmanControl : public ::Kalman::Vector<float, 6> {
        public:
            KALMAN_VECTOR(KalmanControl, float, 6)

            static constexpr ::std::size_t DELTA_X = 0 ;

            inline float& delta_x() noexcept
            {
                return (*this)[DELTA_X] ;
            }

            inline float delta_x() const noexcept
            {
                return (*this)[DELTA_X] ;
            }

            static constexpr ::std::size_t DELTA_Y = 1 ;

            inline float& delta_y() noexcept
            {
                return (*this)[DELTA_Y] ;
            }

            inline float delta_y() const noexcept
            {
                return (*this)[DELTA_Y] ;
            }

            static constexpr ::std::size_t DELTA_Z = 2 ;

            inline float& delta_z() noexcept
            {
                return (*this)[DELTA_Z] ;
            }

            inline float delta_z() const noexcept
            {
                return (*this)[DELTA_Z] ;
            }

            static constexpr size_t DELTA_ROLL = 3 ;

            inline float& delta_roll() noexcept
            {
                return (*this)[DELTA_ROLL] ;
            }

            inline float delta_roll() const noexcept
            {
                return (*this)[DELTA_ROLL] ;
            }

            static constexpr size_t DELTA_PITCH = 4 ;

            inline float& delta_pitch() noexcept
            {
                return (*this)[DELTA_PITCH] ;
            }

            inline float delta_pitch() const noexcept
            {
                return (*this)[DELTA_PITCH] ;
            }

            static constexpr size_t DELTA_YAW = 5 ;

            inline float& delta_yaw() noexcept
            {
                return (*this)[DELTA_YAW] ;
            }

            inline float delta_yaw() const noexcept
            {
                return (*this)[DELTA_YAW] ;
            }

};


    class SystemModel : public ::Kalman::LinearizedSystemModel<KalmanState, KalmanControl, ::Kalman::StandardBase> {
    /**
      * @brief SystemModel (class) - System model for a 6DOF robot
      * This is the system model defining how our robot moves from one
      * time-step to the next, i.e. how the system state evolves over time.
      */
        public:
            /**
              * @brief f - Definition of non-linear state transition function (ie. how the system state is propagated through time)
              * @param const sdr::KalmanState& - The system state in current time-step
              * @return sdr::KalmanState - system state predicted for next time-step
              */
            KalmanState f(const KalmanState& current_step, const KalmanControl& control) const
            {
                KalmanState next_step ; // predicted state vector after transition
                next_step.setZero() ;

                /* create objects based on old values */
                const sdr::position_t old_position = sdr::position_t{current_step.x(), current_step.y(), current_step.z()} ;

                std::cout << "exty: should be 0.174533 on step 2: " << current_step.yaw() << std::endl ;

                const sdr::rotation_m_t current_yaw{
                    {static_cast<float>(std::cos(current_step.yaw())),static_cast<float>(std::asin(current_step.yaw())),0.f}, \
                    {static_cast<float>(std::sin(current_step.yaw())),static_cast<float>(std::cos(current_step.yaw())),0.f}, \
                    {0.f,0.f,1.f}
                } ;

                const sdr::rotation_m_t current_pitch{
                    {static_cast<float>(std::cos(current_step.pitch())),0.f,static_cast<float>(std::sin(current_step.pitch()))}, \
                    {0.f,1.f,0.f}, \
                    {static_cast<float>(std::asin(current_step.pitch())),0.f,static_cast<float>(std::cos(current_step.pitch()))}
                } ;

                const sdr::rotation_m_t current_roll{
                    {1.f,0.f,0.f}, \
                    {0.f,static_cast<float>(std::cos(current_step.roll())),static_cast<float>(std::asin(current_step.roll()))}, \
                    {0.f,static_cast<float>(std::sin(current_step.roll())),static_cast<float>(std::cos(current_step.roll()))}
                } ;

                const sdr::rotation_m_t current_orientation = current_yaw * current_pitch * current_roll ;

                /* calculate new values using old values and predicted control values */
                const sdr::position_t delta_position = sdr::position_t{{control.delta_x(), control.delta_y(), control.delta_y()}} * current_orientation ;
                const sdr::position_t new_position = old_position + delta_position ;
                std::cout << "old pos: " << old_position << std::endl ;
                std::cout << "new pos: " << new_position << std::endl ;
                next_step.x(new_position.coeff(0,0)) ;
                next_step.y(new_position.coeff(0,1)) ;
                next_step.z(new_position.coeff(0,2)) ;

                const sdr::rotation_m_t new_delta_yaw{
                    {static_cast<float>(std::cos(control.delta_yaw())),static_cast<float>(std::asin(control.delta_yaw())),0.f}, \
                    {static_cast<float>(std::sin(control.delta_yaw())),static_cast<float>(std::cos(control.delta_yaw())),0.f}, \
                    {0.f,0.f,1.f}
                } ;

                const sdr::rotation_m_t new_delta_pitch{
                    {static_cast<float>(std::cos(control.delta_pitch())),0.f,static_cast<float>(std::sin(control.delta_pitch()))}, \
                    {0.f,1.f,0.f}, \
                    {static_cast<float>(std::asin(control.delta_pitch())),0.f,static_cast<float>(std::cos(control.delta_pitch()))}
                } ;

                const sdr::rotation_m_t new_delta_roll{
                    {1.f,0.f,0.f}, \
                    {0.f,static_cast<float>(std::cos(control.delta_roll())),static_cast<float>(std::asin(control.delta_roll()))}, \
                    {0.f,static_cast<float>(std::sin(control.delta_roll())),static_cast<float>(std::cos(control.delta_roll()))}
                } ;

                const sdr::rotation_m_t delta_orientation = new_delta_yaw * new_delta_pitch * new_delta_roll ;
                const sdr::rotation_m_t new_orientation = current_orientation * delta_orientation ;
                std::cout << "current orientation: " << current_orientation << std::endl ;
                std::cout << "new orientation: " << new_orientation << std::endl ;

                next_step.yaw(static_cast<float>(std::atan2(new_orientation(1,0), new_orientation(0,0)))) ;
                std::cout << "yaw val extract: " << next_step.yaw() << std::endl ;
//                std::cout << "local: should be 0.174533 on step 1: " << next_step.yaw() << std::endl ; // it is, so it bloody updates locally
                next_step.pitch(std::atan2(-new_orientation(2,0), std::pow(new_orientation(2,1) * new_orientation(2,1) + new_orientation(2,2) * new_orientation(2,2), 0.5))) ;
                next_step.roll(std::atan2(new_orientation(2,1), new_orientation(2,2))) ;

                std::cout << "\nkabfKJAb" << next_step << std::endl ;
                return next_step ;
        }
    } ;

    struct Measurement : public Kalman::Vector<float, 6> {
        public:
            KALMAN_VECTOR(Measurement, float, 6)

            constexpr static ::std::size_t X = 0 ;

            inline float& x() noexcept
            {
                return (*this)[X] ;
            }

            inline float x() const noexcept
            {
                return (*this)[X] ;
            }

            constexpr static ::std::size_t Y = 1 ;

            inline float& y() noexcept
            {
                return (*this)[Y] ;
            }

            inline float y() const noexcept
            {
                return (*this)[Y] ;
            }

            constexpr static ::std::size_t Z = 2 ;

            inline float& z() noexcept
            {
                return (*this)[Z] ;
            }

            inline float z() const noexcept
            {
                return (*this)[Z] ;
            }

            constexpr static ::std::size_t YAW = 3 ;

            inline float& yaw() noexcept
            {
                return (*this)[YAW] ;
            }

            inline ::std::size_t yaw() const noexcept
            {
                return (*this)[YAW] ;
            }

            constexpr static ::std::size_t PITCH = 4 ;

            inline float& pitch() noexcept
            {
                return (*this)[PITCH] ;
            }

            inline float pitch() const noexcept
            {
                return (*this)[PITCH] ;
            }

            constexpr static ::std::size_t ROLL = 5 ;

            inline float& roll() noexcept
            {
                return (*this)[ROLL] ;
            }

            inline float roll() const noexcept
            {
                return (*this)[ROLL] ;
            }

    } ;

    class MeasurementModel : public Kalman::LinearizedMeasurementModel<KalmanState, Measurement, ::Kalman::StandardBase> {
        public:
            MeasurementModel()
            {
                // Setup jacobians. As these are static, we can define them once
                // and do not need to update them dynamically
                this->H.setIdentity();
                this->V.setIdentity();
            }

           /**
             * @brief Definition of (possibly non-linear) measurement function
             * This function maps the system state to the measurement that is expected
             * to be received from the sensor assuming the system is currently in the
             * estimated state.
             * @param [in] x The system state in current time-step
             * @return The (predicted) sensor measurement for the system state
             */
             Measurement h(const KalmanState& current_state) const
             {
                Measurement measurement ;
                measurement.x() = current_state.x() ; measurement.y() = current_state.y() ; measurement.z() = current_state.z() ;
                measurement.yaw() = current_state.yaw() ; measurement.pitch() = current_state.pitch() ; measurement.roll() = current_state.roll() ;
                return measurement ;
             }
    } ;

} ;

#endif // UNSCENTED_KALMAN_SUPPORT_HPP
