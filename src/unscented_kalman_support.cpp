#include "unscented_kalman_support.hpp"

/**
  * @brief File contains definitions of functionality to support and simplify use of external Kalman filter library, specifically the unscented kalman filter
  */

void sdr::DeltaMeasurement::delta(const float x) noexcept
{
    this->delta = x ;
}

float sdr::DeltaMeasurement::delta() const noexcept
{
    return this->delta ;
}
