# INI handler for c++ project

# Include method
Header file download and move to project include folder path

    git clone http://KUAilab.synology.me:30000/ailabtools/ini_handler_cpp.git
    mv ini_handler_cpp/ {$Project Include folder Path}

Include your project source code

    #include "ini_handle_cpp/c_ini.hpp"

# Functions

## Init
Initialization of ini handler class

    CINI_H::Init(std::string ini_file_path);

## Parse ini file
Parsing the ini data.
It need section and key string.
And return the value.

    CINI_H::ParseConfig(std::string section, std::string key, type_T &value);

## Write ini value
TBD