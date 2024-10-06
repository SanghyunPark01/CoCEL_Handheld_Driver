#include "utility.h"

int main(void)
{
    TimeShare* TIME_SHARE;
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

    while(1)
    {
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);  

        int32_t nTestTime = TIME_SHARE->mnTime;

        std::cout << "Time Share Value: " << nTestTime << "\n";
    }

    return 0;
}