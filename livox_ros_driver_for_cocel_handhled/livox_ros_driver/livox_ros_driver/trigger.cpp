/*
Add file by 2024 Sanghyun Park.
E-mail: pash0302@gmail.com
E-mail: pash0302@postech.ac.kr
Github: SanghyunPark01 (https://github.com/SanghyunPark01)

**Added for CoCEL Handheld**
*/

#include "trigger.h"
#include <ros/ros.h>

Trigger::Trigger(){}

bool Trigger::setTrigger(rapidjson::Document &doc)
{
    rapidjson::Value object;

    bool bEnableTrigger = false;
    try
    {
        if(!doc.HasMember("trigger_config") || !doc["trigger_config"].IsObject())
        {
            throw false;
        }
        object = doc["trigger_config"];

        if(!object.HasMember("enable_trigger") || !object["enable_trigger"].IsBool())
        {
            throw false;
        }
        bEnableTrigger = object["enable_trigger"].GetBool();
    }
    catch(bool &e)
    {
        bEnableTrigger = false;
    }
    
    if(!bEnableTrigger)
    {
        std::cout << "\033[1;33mDisable Trigger\033[0m" << "\n";
        return false;
    }
    else
    {
        std::cout << "\033[1;33mEnable Trigger\033[0m" << "\n";
    }

    std::cout << "\n";
    std::cout << "\033[1;32m    ┌────────────────────────────────────────┐\033[0m" << "\n";
    std::cout << "\033[1;32m┌───┤ Triggering code added by Sanghyun Park ├───┐\033[0m" << "\n";
    std::cout << "\033[1;32m│   └────────────────────────────────────────┘   │\033[0m" << "\n";
    std::cout << "\033[1;32m│**Contact**                                     │\033[0m" << "\n";
    std::cout << "\033[1;32m│E-mail: pash0302@gmail.com                      │\033[0m" << "\n";
    std::cout << "\033[1;32m│E-mail: pash0302@postech.ac.kr                  │\033[0m" << "\n";
    std::cout << "\033[1;32m└────────────────────────────────────────────────┘\033[0m" << "\n";
    std::cout << "\n";
    std::cout << "\033[1;33mInitialize Trigger...\033[0m" << "\n\n";

    // Set Timeshare
    std::string OUTPUT_PATH = "/home/" + std::string(getlogin()) + "/timeshare";
    int fd = open(OUTPUT_PATH.c_str(), O_RDWR);
    if (fd == -1) {
        ROS_ERROR("Can't Open TimeShare File");
        return false;
    } else {
        std::cout << "\033[1;33mOpen Timeshare File: " << OUTPUT_PATH << "\033[0m\n";
    }
    TIME_SHARE = (TimeShare *)mmap(NULL, sizeof(TimeShare), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    std::cout << "\033[1;32mRead TimeShare Memory Successfuly\033[0m" << "\n\n";

    
    // Set UART
    
    try
    {

        if (!object.HasMember("device_name") || !object["device_name"].IsString())
        {
            throw false;
        }
        std::string device_name = object["device_name"].GetString();
        std::strncpy(_mTriggerConfig.dev_config.name, device_name.c_str(), sizeof(_mTriggerConfig.dev_config.name));

        if (!object.HasMember("comm_device_type") || !object["comm_device_type"].IsInt())
        {
            throw false;
        }
        _mTriggerConfig.dev_config.type = object["comm_device_type"].GetInt();

        if (_mTriggerConfig.dev_config.type == livox_ros::kCommDevUart)
        {
            if (!object.HasMember("baudrate_index") || !object["baudrate_index"].IsInt())
            {
                throw false;
            }
            _mTriggerConfig.dev_config.config.uart.baudrate =object["baudrate_index"].GetInt();

            if (!object.HasMember("parity_index") || !object["parity_index"].IsInt())
            {
                throw false;
            }            
            _mTriggerConfig.dev_config.config.uart.parity = object["parity_index"].GetInt();
        }
        else
        {
            std::cout << "\033[1;31m[CONFIG]Only use UART!\033[0m" << "\n";
            throw false;
        }

        // Open UART
        if(!initUART(_mTriggerConfig))
        {
            throw false;
        }
    }
    catch(bool &e)
    {
        std::cout << "\033[1;31mFailed to initialize trigger\033[0m" << "\n\n";
        return false;
    }
   
    std::cout << "\033[1;33mInitialize Success!\033[0m" << "\n\n";
    return true;
}

bool Trigger::initUART(const TriggerConfig &config)
{
    if(config.dev_config.type != livox_ros::kCommDevUart)
    {
        std::cout << "\033[1;31mOnly use UART!\033[0m" << "\n";
        return false;
    }
    
    uint8_t baudrate_index = config.dev_config.config.uart.baudrate;
    uint8_t parity_index = config.dev_config.config.uart.parity;

    if ((baudrate_index >= livox_ros::BRUnkown) || (parity_index >= livox_ros::ParityUnkown))
    {
        std::cout << "\033[1;31mUart parameter error, please check the configuration file!\033[0m" << "\n";
        return false;
    }
    _mptrUART = new livox_ros::UserUart(baudrate_index, parity_index);
    
    if(_mptrUART->Open(config.dev_config.name) == -1)
    {
        std::cout << "\033[1;31mCan't Open UART, please check UART!\033[0m" << "\n";
        return false;
    }

    return true;
}

void Trigger::sendFlag2UART(int32_t nTimeDiff)
{
    if(!_mptrUART->IsOpen())
    {
        std::cout << "\033[1;31mCan't Connect UART, please check UART!\033[0m" << "\n";
        return;
    }

    int32_t nSendTime = TIME_SHARE->mnTime; // [us]
    nSendTime = nSendTime/10; // [dus]
    std::cout << "send to stm: " << nTimeDiff << ", " << nSendTime << "\n";
    // std::cout << "send to stm: " << _mnCnt++ << "\n";
    // int32_t msg[1] = {nSendTime};
    int32_t msg[2] = {nTimeDiff, nSendTime};

    // char msg0[1] = {'1'};
    // char msg1[1] = {'0'};
    // if (_mnCnt%2 == 0)
    // {
    //     _mptrUART -> Write(msg0, sizeof(msg0));
    // }
    // else
    // {
    //     _mptrUART -> Write(msg1, sizeof(msg1));
    // }   
    // _mptrUART -> Write(msg0, sizeof(msg0));

    _mptrUART -> WriteINT32(msg, sizeof(msg));
}
