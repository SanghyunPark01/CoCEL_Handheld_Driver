# include "time_delay_estimator.h"

Estimator::Estimator(const ros::NodeHandle& nh_):_nh(nh_)
{
    _nh.param<std::string>("/topic_name/lidar", LIDAR_TOPIC, "/");
    _nh.param<std::string>("/topic_name/image", IMG_TOPIC, "/");

    _nh.param<int>("/topic_type/lidar", LIDAR_TYPE, 0);
    _nh.param<int>("/topic_type/image", IMG_TYPE, 0);

    _nh.param<int>("/estimation_type", ESTIMATION_TYPE, 0);
    if(ESTIMATION_TYPE == 0)
    {
        std::cout << "\n\033[1;32mEstimation Type: Kalman Filter\033[0m" << "\n\n";
    }
    else if(ESTIMATION_TYPE == 1)
    {
        std::cout << "\n\033[1;32mEstimation Type: Test\033[0m" << "\n\n";
    }
    else
    {
        ROS_ERROR("Invalid Estimation type");
    }

    setupTimeShare();

    // Config LiDAR Subscriber
    if(LIDAR_TYPE == 0)
    {
        ROS_ERROR("Not support lidar type");
    }
    else if(LIDAR_TYPE == 1)
    {
        std::cout << "\033[1;32mLiDAR Type: livox custom msg\033[0m" << "\n";
        _mSubLiDAR = _nh.subscribe(LIDAR_TOPIC, 10, &Estimator::callbackCustumLiDAR, this);
    }
    else ROS_ERROR("Invalid lidar type");

    // Config Image Subscriber
    if(IMG_TYPE == 0)
    {
        ROS_ERROR("Not support image type");
    }
    else if(IMG_TYPE == 1)
    {
        std::cout << "\033[1;32mImage Type: compressed image\033[0m" << "\n";
        _mSubImg = _nh.subscribe(IMG_TOPIC, 30, &Estimator::callbackCompressedImg, this);
    }
    else ROS_ERROR("Invalid image type");
    std::cout << "\n";
}

/* @@@@@@@@@@@@@@@@@@@@@@@@@@
@@@ ROS CALLBACK FUNCTION @@@
@@@@@@@@@@@@@@@@@@@@@@@@@@ */
void Estimator::callbackCustumLiDAR(const livox_ros_driver::CustomMsg::ConstPtr &msgLiDAR)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackLiDAR);
    _mqLiDARStampBuf.push(msgLiDAR->header.stamp.toSec());
}
void Estimator::callbackCompressedImg(const sensor_msgs::CompressedImageConstPtr &msgImg)
{
    std::unique_lock<std::mutex> lock(_mtxCallbackImg);
    _mqImgStampBuf.push(msgImg->header.stamp.toSec());
}

/* @@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@ THREAD @@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@ */
void Estimator::syncTime(void)
{
    while (1)
    {
        // std::chrono::milliseconds dura(2);
        // std::this_thread::sleep_for(dura);
        std::pair<double,double> pairTime;
        bool bStatus = getTimePair(pairTime);
        if(!bStatus)continue;

        validateTimePair(pairTime);
    }
}
void Estimator::estimateDelay(void)
{
    while (1)
    {
        // std::chrono::milliseconds dura(2);
        // std::this_thread::sleep_for(dura);
        double dTimeDiff = 0;;
        double dLiDARdt = 0; 
        bool status = getTimeSync(dTimeDiff, dLiDARdt);
        if(!status)continue;

        double dEstimatedDelay = 0;
        if(ESTIMATION_TYPE == 0)
        {
            dEstimatedDelay = estimateTimeDelayKF(dTimeDiff, dLiDARdt);
        }
        else if(ESTIMATION_TYPE == 1)
        {
            dEstimatedDelay = estimateTimeDelayTest(dTimeDiff, dLiDARdt);
        }
         
        // Time share
        int32_t nTime = dEstimatedDelay*1e6; // to us
        TIME_SHARE->mnTime = nTime;
    }
}

/* @@@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@ FOR THREAD @@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@ */
bool Estimator::getTimePair(std::pair<double,double> &pairTime)
{
    if(_mqLiDARStampBuf.empty() || _mqImgStampBuf.empty())return false;

    // get time
    double dLiDARTime, dImageTime;
    {
        std::unique_lock<std::mutex> lockLiDAR(_mtxCallbackLiDAR);
        std::unique_lock<std::mutex> lockImage(_mtxCallbackImg);
        dLiDARTime = _mqLiDARStampBuf.front();
        dImageTime = _mqImgStampBuf.front();
    }

    // make pair
    pairTime.first = dLiDARTime;
    pairTime.second = dImageTime;

    return true;
}
void Estimator::validateTimePair(std::pair<double,double> timepair)
{
    double dLiDARTime = timepair.first;
    double dImageTime = timepair.second;
    _mdCurrImgTime = dImageTime;
    _mdCurrLiDARTime = dLiDARTime;
    // pop image if...
    if(_mdPrevImgTime < dLiDARTime && _mdCurrImgTime < dLiDARTime)
    {
        std::unique_lock<std::mutex> lockImage(_mtxCallbackImg);
        _mqImgStampBuf.pop();
        _mdPrevImgTime = _mdCurrImgTime;
        return;
    }
    // pop lidar if...
    else if(_mdPrevImgTime > dLiDARTime && _mdCurrImgTime > dLiDARTime)
    {
        std::unique_lock<std::mutex> lockLiDAR(_mtxCallbackLiDAR);
        _mqLiDARStampBuf.pop();
        _mdPrevLiDARTime = _mdCurrLiDARTime;
        return;
    }

    /* --------------------------------------------------------------
        curr status: _mdPrevImgTime < dLiDARTime < _mdCurrImgTime
    -------------------------------------------------------------- */
    // check closer timestamp
    std::tuple<double,double,double> pairValidTime;
    double dClosestImgTime = (_mdCurrImgTime - dLiDARTime) > (dLiDARTime - _mdPrevImgTime) ? _mdPrevImgTime : _mdCurrImgTime;
    double dLiDARTimeDiff = _mdCurrLiDARTime - _mdPrevLiDARTime;

    _mdPrevImgTime = _mdCurrImgTime;
    _mdPrevLiDARTime = _mdCurrLiDARTime;

    // pop and push
    pairValidTime = std::make_tuple(dLiDARTime, dClosestImgTime, dLiDARTimeDiff);
    {
        std::unique_lock<std::mutex> lockLiDAR(_mtxCallbackLiDAR);
        std::unique_lock<std::mutex> lockImage(_mtxCallbackImg);
        _mqLiDARStampBuf.pop();
        _mqImgStampBuf.pop();
    }
    {
        std::unique_lock<std::mutex> lockTime(_mtxTime);
        _mqTimePairBuf.push(pairValidTime);
    }
}
bool Estimator::getTimeSync(double &dTimeDiff, double &dLiDARdt)
{
    if(_mqTimePairBuf.empty())return false;

    std::tuple<double,double,double> pairTimeSync;
    {
        std::unique_lock<std::mutex> lockTime(_mtxTime);
        pairTimeSync = _mqTimePairBuf.front();
        _mqTimePairBuf.pop();
    }

    double dDiffTime = std::get<0>(pairTimeSync) - std::get<1>(pairTimeSync);
    double dt = std::get<2>(pairTimeSync);

    dTimeDiff = dDiffTime;
    dLiDARdt = dt;

    return true;
}
double Estimator::estimateTimeDelayKF(double dTimeDiff, double dLiDARdt)
{
    // std::cout << "Lidar - Camera: " << dTimeDiff << ", dt = " << dLiDARdt << "\n";

    if(_mnKFInit > 0)
    {
        _mnKFInit--;
        return 0;
    }

    // Initialize Kalman Filter
    if(!_mbKFInit)
    {
        Eigen::Vector2d INITIAL_STATE(dTimeDiff, 0);
        _mptrKF = new KalmanFilter(PROCESS_VAR, MEASUREMENT_VAR, INITIAL_STATE);
        _mbKFInit = true;
        std::cout << "\033[1;32mKalman Filter Init Success!!\033[0m" << "\n";
        return 0;
    } 
    
    TicToc time;

    _mptrKF->predictState(dLiDARdt);
    _mptrKF->updateState(dTimeDiff);

    // double dPredictedNextState = _mptrKF->getCurrentState();
    double dPredictedNextState = _mptrKF->predictNextState();

    double dProcessingTime = time.toc();

    double dLoss = dTimeDiff - _mdPreviousPrediction;
    _mdPreviousPrediction = dPredictedNextState;
    std::cout << "=======================================\n";
    std::cout << "Predicted Next State: " << dPredictedNextState << ", KF Processing time: " << dProcessingTime << "[ms]\n";
    std::cout << "Residual of GT and previous next status prediction: " << dLoss << "\n";
    
    return dPredictedNextState;
}

double Estimator::estimateTimeDelayTest(double dTimeDiff, double dLiDARdt)
{
    double tmpTimeDiff = abs(dTimeDiff*1e5); // to dus
    if(!_mbIsConverge && tmpTimeDiff < 20)
    {
        std::cout << "\033[1;32mConverge Time Delay Success!!\033[0m" << "\n";
        _mbIsConverge = true;
    }
    // if(_mbIsConverge && abs(tmpTimeDiff) > 180)
    // {
    //     std::cout << "\033[1;31mConverge Time Delay Fail!!\033[0m" << "\n";
    //     _mbIsConverge = false;
    //     _mError.clear();
    // }
    
    if(_mbIsConverge)
    {
        // 
        _mdTotalPair++;
        _mdMaxError = tmpTimeDiff/100 > _mdMaxError ? tmpTimeDiff/100 : _mdMaxError;
        _mdMinError = tmpTimeDiff/100 < _mdMinError ? tmpTimeDiff/100 : _mdMinError;
        _mdSumError += tmpTimeDiff/100;
        if(tmpTimeDiff > 100)_mdFailNum++;

        _mError.push_back(tmpTimeDiff);
        std::cout << "========================================================\n";
        std::cout << "Sampling Num: " << _mdTotalPair << "\n";
        std::cout << "Total Average Error: " << _mdSumError/_mdTotalPair << "[ms]\n";
        std::cout << "Sliding Window Average of Error(size: " << _mError.size() << "): " << _mError.mean()/100 << "[ms]\n";
        std::cout << "Max Error: " << _mdMaxError << "[ms]\n";
        std::cout << "Min Error: " << _mdMinError << "[ms]\n";
        std::cout << "Fail Num: " << _mdFailNum << "\n";
    }


    return dTimeDiff;
}