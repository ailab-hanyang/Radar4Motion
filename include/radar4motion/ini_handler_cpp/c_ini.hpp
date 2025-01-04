/**
 * @copyright (c) AI LAB - Hanyang Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com dudfhe3349@gmail.com
 * @file c_ini.hpp
 * @brief ini_parser
 * @version 1.0
 * @date 13-02-2022
 * @bug No known bugs
 * @warning No warnings
 */
#ifndef __C_INI_HPP__
#define __C_INI_HPP__

#include "ini.h"
#include <iostream>
#include <string>
#include <stdint.h>
#include <math.h>

class CINI_H
{
public:
    CINI_H(){}
    ~CINI_H(){}

public:
    inih::INIReader m_ini_h;
    std::string m_str_ini_path;
    time_t m_fileUpdateTime=-1;

    bool Init(std::string str_path)
    {
        try{
            inih::INIReader ini_h{str_path};
            m_ini_h = ini_h;
            m_str_ini_path = str_path;
        }
        catch(std::exception &err)
        {
            std::cout << "\033[1m\033[31m" << typeid(err).name() << "\033[0m" << std::endl;
            std::cerr << "\033[1m\033[31m" << err.what() << "\033[0m" << std::endl;
            std::cout << "\033[1m\033[31m" << "[Ini handler] Initialization error " << "\033[0m" << std::endl;
            return false;
        }
        return true;
    }
    bool IsFileUpdated()
    {
        struct stat attr;
        if (stat( m_str_ini_path.c_str(), &attr) == -1) return false;
        if ((attr.st_mtime == m_fileUpdateTime) && (m_fileUpdateTime != -1)) return false;
        m_fileUpdateTime = attr.st_mtime;
        Init(m_str_ini_path);
        return true;
    }
    
    template<typename type_T>
    bool ParseConfig(std::string section, std::string key, type_T &value)
    {
        try{value = m_ini_h.Get<type_T>(section, key);}
        catch(std::exception &err)
        {
            std::cout << "\033[1m\033[31m" << typeid(err).name() << "\033[0m" << std::endl;
            std::cerr << "\033[1m\033[31m" << err.what() << "\033[0m" << std::endl;
            return false;
        }
        return true;
    }
    
    template<typename type_T>
    bool ParseConfig(std::string section, std::string key, std::vector<type_T> &value)
    {
        try{value = m_ini_h.GetVector<type_T>(section, key);}
        catch(std::exception &err){
            std::cout << "\033[1m\033[31m" << typeid(err).name() << "\033[0m" << std::endl;
            std::cerr << "\033[1m\033[31m" << err.what() << "\033[0m" << std::endl;
            return false;
        }
        return true;
    }

    // Need to make insert configuration
    // template<typename type_T>
    // bool InsertConfig(std::string section, std::string key, type_t va_list)
    // {
    //     return ture;
    // }
};

#endif //__C_INI_HPP__