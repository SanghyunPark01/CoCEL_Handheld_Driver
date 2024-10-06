#ifndef TIME_DELAY_ESTIMATION
#define TIME_DELAY_ESTIMATION

#include "utility.h"
#include "kalman_filter.h"

class Estimator
{
private:
    // param
    std::string LIDAR_TOPIC, IMG_TOPIC;
    int LIDAR_TYPE, IMG_TYPE;
    int ESTIMATION_TYPE;
    std::string OUTPUT_PATH;
    TimeShare *TIME_SHARE;

    void setupTimeShare(void)
    {
        OUTPUT_PATH = "/home/" + std::string(getlogin()) + "/timeshare";
        int fd = open(OUTPUT_PATH.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (fd == -1) {
            ROS_ERROR("Can't Open Timshare File");
            return;
        } else {
            std::cout << "\033[1;33mOpen Timeshare File: " << OUTPUT_PATH << "\033[0m\n";
        }
        lseek(fd, sizeof(TimeShare) * 1, SEEK_SET);
        write(fd, "", 1);
        TIME_SHARE = (TimeShare *)mmap(NULL, sizeof(TimeShare) * 1, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        std::cout << "\033[1;32mAllocate TimeShare Memory Successfuly\033[0m" << "\n\n";
    }

    // ROS
    ros::NodeHandle _nh;
    ros::Subscriber _mSubLiDAR;
    ros::Subscriber _mSubImg;
    ros::Publisher _mPubSyncStatus;

    // ROS callback
    void callbackCustumLiDAR(const livox_ros_driver::CustomMsg::ConstPtr &msgLiDAR);
    void callbackCompressedImg(const sensor_msgs::CompressedImageConstPtr &msgImg);

    // ROS data
    std::queue<double> _mqLiDARStampBuf;
    std::mutex _mtxCallbackLiDAR;
    std::queue<double> _mqImgStampBuf;
    std::mutex _mtxCallbackImg;

    // Time sync pair
    std::queue<std::tuple<double,double,double>> _mqTimePairBuf;
    std::mutex _mtxTime;
    double _mdPrevImgTime = 0;
    double _mdCurrImgTime = 0;
    double _mdPrevLiDARTime = 0;
    double _mdCurrLiDARTime = 0;
    bool getTimePair(std::pair<double,double> &pairTime);
    void validateTimePair(std::pair<double,double> pairTime);

    // Delay Estimation
    bool getTimeSync(double &dTimeDiff, double &dLiDARdt);
    // 
    double estimateTimeDelayKF(double dTimeDiff, double dLiDARdt);
    double estimateTimeDelayTest(double dTimeDiff, double dLiDARdt);
    bool _mbIsConverge = false;

    double _mdMaxError = 0;
    double _mdMinError = 9999;
    double _mdSumError = 0;  // for full mean
    double _mdTotalPair = 0;
    double _mdFailNum = 0;
    SlidingWindow<double> _mError = SlidingWindow<double>(10);

    // KalmanFilter
    double PROCESS_VAR = 1e-3;
    double MEASUREMENT_VAR = 1e-2;
    KalmanFilter* _mptrKF;
    int _mbKFInit = false;
    int _mnKFInit = 1;
    double _mdPreviousPrediction = 0;

    // For debugging
    double _mdDebug = 9999;
public:
    // Constructor
    Estimator(const ros::NodeHandle& nh_);
    
    // Thread
    void syncTime(void);
    void estimateDelay(void);

    // Reset
    void resetTime(void)
    {
        TIME_SHARE->mnTime = 0;
    }
};


#endif TIME_DELAY_ESTIMATION