#include "time_delay_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feedback_sensor_time");
    ros::NodeHandle nh_("~");

    Estimator TimeDelayEstimator(nh_);

    std::thread thSyncTime(&Estimator::syncTime, &TimeDelayEstimator);
    std::thread thEstimation(&Estimator::estimateDelay, &TimeDelayEstimator);

    ros::spin();

    signal(SIGINT, mySigintHandler);
    ros::waitForShutdown();

    if(!ros::ok())
    {
        TimeDelayEstimator.resetTime();
        std::cout << "Shut down feedback_sensor_time!" << "\n";
    }

    return 0;
}