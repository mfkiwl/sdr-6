#ifndef DETAILED_EXCEPTION_HPP
#define DETAILED_EXCEPTION_HPP
#pragma once

#include <stdexcept>
#include <string>

/**
  * @brief File storing declarations for exception class which gives more detailed exception information beyond a simple message
  */

namespace sdr {

    class DetailedException : public std::exception {
    /**
      * @brief DetailedException (class) - class to more easily provide detailed exception messages, such as function name
      */
        private:
            std::string msg_ ;

        public:
            /**
              * @brief DetailedException (constructor) - initialises message for exception
              * @param const std::string& - const lvalue reference to string storing function name
              * @param const unsigned int - const number storing line number
              * @param const std::string& - const lvalue reference to string storing message to be displayed upon being thrown about
              */
            explicit DetailedException(const std::string&, const unsigned int, const std::string&) noexcept(false) ;

            /**
              * @brief DetailedException (copy constructor) - initialises message for exception
              * @param const DetailedException& - const lvalue reference detailed exception class storing error message properties
              */
            explicit DetailedException(const DetailedException&) noexcept(false) ;

            /**
              * @brief Assignment operator - copy assignment takes messages for exception
              * @param const DetailedException& - const lvalue reference to detailed exception class storing error message properties
              * @return DetailedException& - initialised error throwing class
              */
            DetailedException& operator=(const DetailedException&) noexcept(false) ;

            /**
              * @brief DetailedException (move constructor) - initialises message for exception
              * @param DetailedException&& - rvalue reference to detailed exception class storing error message properties
              */
            explicit DetailedException(DetailedException&&) noexcept ;

            /**
              * @brief Assignment operator - move assignment takes messages for exception
              * @param DetailedException&& - rvalue reference to detailed exception class storing error message properties
              * @return DetailedException& - lvalue reference to initialised error throwing class
              */
            DetailedException& operator=(DetailedException&&) noexcept ;

            /**
              * @brief what - function returning error message
              * @return const char* - cstring storing message to be displayed upon being thrown
              */
            virtual const char* what() const noexcept ;

            /**
              * @brief ~DetailedException (destructor) - function releases memory of values held
              */
            virtual ~DetailedException() noexcept ;

            // DELETED DetailedException (constructor) - requires messages / details of exception. THATS THE POINT
            DetailedException() = delete ;
    } ;

} ;

#endif // DETAILED_EXCEPTION_HPP
