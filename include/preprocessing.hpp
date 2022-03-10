#ifndef PREPROCESSING_HPP
#define PREPROCESSING_HPP
#pragma once

#include <string>
#include <filesystem>

#include "pose.hpp"

/**
  * @brief File contains the declarations related to preprocessing necessary data for salih slam system
  */

namespace sdr {

    /**
      * @brief is_meta_file (overload) - determines whether given name of path is to a file
      * @param const std::string& - const lvalue reference to string storing path name to test
      * @return bool - whether given path leads to directory
      */
    bool is_meta_file(const std::string&) noexcept ;

    /**
      * @brief is_meta_file (overload) - determines whether given name of path is to a file
      * @param const std::filesystem::path& - const lvalue reference to filesystem object initialised to path
      * @return bool - whether given path leads to directory
      */
    bool is_meta_file(const std::filesystem::path&) noexcept ;

    /**
      * @brief is_meta_yaml (overload) - determines whether given name of path is to a YAML file
      * @param const std::string& - const lvalue reference to string storing path name to test
      * @return bool - whether given path leads to file with YAML file
      */
    bool is_meta_yaml(const std::string&) noexcept ;

    /**
      * @brief is_meta_yaml (overload) - determines whether given name of path is to a YAML file
      * @param const std::filesystem::path& - const lvalue reference to filesystem object initialised to path
      * @return bool - whether given path leads to file with YAML file
      */
    bool is_meta_yaml(const std::filesystem::path&) noexcept ;

    /**
      * @brief extract_initial_pose - extracts information to where the car starts in a map (its initial pose)
      * @param const std::strng& - const reference to string name relating to file path of config file
      * @throws sdr::DetailedException - thrown when file to read from isn't in YAML format, when translation matrix size is not valid, when orientation matrix size is not valid
      * @return sdr::Pose - specified initial pose information
      */
    Pose extract_initial_pose(const std::string&) noexcept(false) ;

}

#endif // PREPROCESSING_HPP
