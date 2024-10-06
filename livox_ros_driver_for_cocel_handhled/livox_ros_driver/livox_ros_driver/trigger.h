/*
Add file by 2024 Sanghyun Park.
E-mail: pash0302@gmail.com
E-mail: pash0302@postech.ac.kr
Github: SanghyunPark01 (https://github.com/SanghyunPark01)

**Added for CoCEL Handheld**
*/

#ifndef TRIGGER_H_
#define TRIGGER_H_

#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "../common/rapidjson/document.h"
#include "../common/comm/comm_device.h"
#include "../common/comm/comm_protocol.h"
#include "../timesync/user_uart/user_uart.h"

struct TimeShare{
    int32_t mnTime = 0;
};

typedef struct 
{
    livox_ros::CommDevConfig dev_config;
    // livox_ros::ProtocolConfig protocol_config;
} TriggerConfig;

class Trigger 
{
public:
    Trigger();
    bool setTrigger(rapidjson::Document &doc);
    bool getStatus(void){return _mbIsEnable;}
    void sendFlag2UART(int32_t nTimeDiff);
private:
    bool _mbIsEnable = false;
    TriggerConfig _mTriggerConfig;
    livox_ros::UserUart *_mptrUART = nullptr;
    int _mnCnt = 0;

    TimeShare* TIME_SHARE;

    bool initUART(const TriggerConfig &config);
};

#endif TRIGGER_H_