#include <filesystem>
#include <string>
#include <vector>
#include <iostream>
#include <cstddef>
#include <stdexcept>
#include <tuple>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "yaml-cpp/yaml.h"

#include "detailed_exception.hpp"
#include "pose.hpp"
#include "preprocessing.hpp"

/**
  * @brief File contains the definitions related to preprocessing necessary data for sdr system
  */

bool sdr::is_meta_file(const std::string& file_name) noexcept
{
    const std::filesystem::path file_object(file_name) ;
    return sdr::is_meta_file(file_object) ;
}

bool sdr::is_meta_file(const std::filesystem::path& file_object) noexcept
{
    if(!std::filesystem::is_regular_file(file_object))
    {
        return false ;
    }
    else if(std::filesystem::is_symlink(file_object))
    {
        std::error_code error_code ;
        const std::filesystem::path re_file_object = std::filesystem::read_symlink(file_object, error_code) ;
        if(!error_code || !std::filesystem::is_regular_file(re_file_object))
        {
            return false ;
        }
    }

    return true ;
}

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
    auto extract_matrix = [config](const std::string& property) -> std::vector<double> {

        std::size_t row_num, col_num ;
        YAML::Node mat_data ;
        try {
            YAML::Node matrix_cont = config[property] ;

            col_num = matrix_cont["cols"].as<std::size_t>() ;
            row_num = matrix_cont["rows"].as<std::size_t>() ;

            mat_data = matrix_cont["data"] ;
        }
        catch(const std::runtime_error& err)
        {
            throw sdr::DetailedException(__func__, __LINE__, err.what()) ;
        }

        std::vector<double> values(col_num * row_num) ;
        for(std::size_t col = 0 ; col < col_num ; ++col)
        {
            for(std::size_t row = 0 ; row < row_num ; ++row)
            {
                values[row+col] = mat_data[col+row].as<double>() ;
            }
        }
        return values ;
    } ;

    sdr::position_t initial_position = Eigen::Map<decltype(initial_position)>(extract_matrix("position").data()) ;
    sdr::quaternion_t initial_quaternion = Eigen::Map<decltype(initial_quaternion)>(extract_matrix("orientation").data()) ;

    return sdr::Pose{initial_position, initial_quaternion} ;
}
