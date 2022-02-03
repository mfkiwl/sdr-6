#include <stdexcept>
#include <string>
#include <utility>

#include "detailed_exception.hpp"

/**
  * @brief File storing definitions for exception class which gives more detailed exception information beyond a simple message
  */

sdr::DetailedException::DetailedException(const std::string& func_name, const unsigned int line_num, const std::string& err_msg) noexcept(false)
{
    this->msg_ = std::string("Error in '") + func_name + std::string("' function, on line ") + std::to_string(line_num) + std::string(": ") + err_msg ;
}

sdr::DetailedException::DetailedException(const sdr::DetailedException& dec) noexcept(false)
{
    this->msg_ = dec.msg_ ;
}

sdr::DetailedException& sdr::DetailedException::operator=(const sdr::DetailedException& dec) noexcept(false)
{
    this->msg_ = dec.msg_ ;
    return *this ;
}

sdr::DetailedException::DetailedException(sdr::DetailedException&& dec) noexcept
{
    this->msg_ = std::move(dec.msg_) ;
    dec.msg_ = nullptr ;
}

sdr::DetailedException& sdr::DetailedException::operator=(sdr::DetailedException&& dec) noexcept
{
    this->msg_ = std::move(dec.msg_) ;
    return *this ;
}

const char* sdr::DetailedException::what() const noexcept
{
    return this->msg_.c_str() ;
}

sdr::DetailedException::~DetailedException() noexcept {} ;
