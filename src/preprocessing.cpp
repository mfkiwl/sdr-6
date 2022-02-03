#include <filesystem>
#include <string>
#include <vector>
#include <iostream>
#include <cstddef>
#include <stdexcept>
#include <tuple>

#include <Eigen/Dense>

#include "yaml-cpp/yaml.h"

#include "detailed_exception.hpp"
#include "pose.hpp"
#include "preprocessing.hpp"

/** @brief File contains the definitions related to preprocessing necessary data for sdr system **/

bool sdr::is_meta_yaml(const std::string& file_name) noexcept
{
    std::filesystem::path file_object(file_name) ;
    return sdr::is_meta_yaml(file_object) ;
}

bool sdr::is_meta_yaml(const std::filesystem::path& file_obj) noexcept
{
    if(!std::filesystem::is_regular_file(file_obj))
    {
        return false ;
    }
    else if(std::filesystem::is_symlink(file_obj))
    {
        std::error_code error_code ;
        const std::filesystem::path re_file_object = std::filesystem::read_symlink(file_obj, error_code) ;
        if(!error_code || std::filesystem::is_directory(re_file_object) || re_file_object.extension().string() == ".yml" || re_file_object.extension().string() == ".yaml")
        {
            return false ;
        }
    }

    return true ;
}

sdr::Pose sdr::extract_initial_pose(const std::string& config_file) noexcept(false)
{
    if(!sdr::is_meta_yaml(config_file))
    {
        const std::string msg = std::string("Initial information file '") + config_file + std::string("' is not a valid YAML file") ;
        throw sdr::DetailedException(__func__, static_cast<unsigned int>(__LINE__), msg) ;
    }

    const YAML::Node config = YAML::LoadFile(config_file) ;

    sdr::position_t initial_translation ;
    {
        YAML::Node matrix_cont = config["position"] ;
        const auto col_num = matrix_cont["cols"].as<std::size_t>() ;
        const auto row_num = matrix_cont["rows"].as<std::size_t>() ;

        if(col_num != 3 || row_num != 1)
        {
            const std::string msg{"Translation matrix dimensions provided are invalid. Required: 1*3. Provided: " + std::to_string(row_num) + "*" + std::to_string(col_num)} ;
            throw sdr::DetailedException(__func__, __LINE__, msg) ;
        }

        const auto mat_data = matrix_cont["data"] ;
        for(std::size_t col = 0 ; col < col_num ; ++col)
        {
            for(std::size_t row = 0 ; row < row_num ; ++row)
            {
                initial_translation(row, col) = mat_data[col+row].as<float>() ;
            }
        }
    }

    sdr::orientation_t initial_orientation ;
    {
        YAML::Node matrix_cont = config["orientation"] ;
        const auto col_num = matrix_cont["cols"].as<std::size_t>() ;
        const auto row_num = matrix_cont["rows"].as<std::size_t>() ;

        if(col_num != 3 || row_num != 1)
        {
            const std::string msg{"Orientation (rotation) matrix dimensions provided are invalid. Required: 3*3. Provided: " + std::to_string(row_num) + "*" + std::to_string(col_num)} ;
            throw sdr::DetailedException(__func__, __LINE__, msg) ;
        }

        const auto mat_data = matrix_cont["data"] ;
        for(std::size_t row = 0 ; row < row_num ; ++row)
        {
            for(std::size_t col = 0 ; col < col_num ; ++col)
            {
                initial_orientation(row, col) = mat_data[col+row].as<float>() ;
            }
        }
    }

    return sdr::Pose{initial_translation, initial_orientation} ;
}
