#ifndef UTILITY_H
#define UTILITY_H

#include <signal.h>
#include <thread>
#include <mutex>
#include <queue>
#include <iomanip>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <fcntl.h>
#include <sys/mman.h>
#include <type_traits>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float64.h>

#include <livox_ros_driver/CustomMsg.h>

struct TimeShare{
    int32_t mnTime = 0;
};

void mySigintHandler(int sig){
    ros::shutdown();
}

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        _mTimeStart = std::chrono::system_clock::now();
    }

    double toc()
    {
        _mTimeEnd = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = _mTimeEnd - _mTimeStart;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> _mTimeStart, _mTimeEnd;
};

template<typename Type>
class SlidingWindow
{
private:
    Type* _mArray;
    int _mnArrSize = 0;
    int _mnCurrSize = 0;
    Type _mSum = Type();
    bool _mbEnableCalculate = false;
public:
    SlidingWindow(int nSize)
    {
        _mnArrSize = nSize;
        _mArray = new Type[nSize];
        if(std::is_same<Type, double>::value || std::is_same<Type, int>::value)
        {
            _mSum = 0;
            _mbEnableCalculate = true;
        }
    }
    ~SlidingWindow()
    {
        delete [] _mArray;
    }
    Type operator[](int nIdx)
    {
        if(nIdx >= _mnCurrSize || nIdx < 0)
        {
            std::cout << "Invalid Index\n";
        }
        return _mArray[nIdx];
    }
    void push_back(Type var)
    {
        // not full
        if(_mnCurrSize < _mnArrSize)
        {
            _mArray[_mnCurrSize] = var;
            _mnCurrSize++;
            if(_mbEnableCalculate)
            {
                _mSum += var;
            }
        }
        // full
        else if(_mnCurrSize == _mnArrSize)
        {
            if(_mbEnableCalculate)
            {
                _mSum -= _mArray[0];
                _mSum += var;
            }
            for(int i = 0; i < _mnArrSize - 1; i++)
            {
                _mArray[i] = _mArray[i + 1];
            }
            _mArray[_mnArrSize - 1] = var;
        }
    }
    void clear()
    {
        _mnCurrSize = 0;
        if(_mbEnableCalculate)
        {
            _mSum = 0;
        }
    }
    int size()
    {
        return _mnCurrSize;
    }
    bool empty()
    {
        if(_mnCurrSize == 0)return true;
        else false;
    }
    Type mean()
    {
        // if (this->empty())return 0;
        if(_mbEnableCalculate)return _mSum/_mnCurrSize;
        else return 0;
    }
};



#endif UTILITY_H