#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>

int mcu_fd = -1;

int spiTxRx(uint8_t txDat) {
    uint8_t rxDat;

    struct spi_ioc_transfer spi{};

    spi.tx_buf = (unsigned long)&txDat;
    spi.rx_buf = (unsigned long)&rxDat;
    spi.len = 1;

    ioctl(mcu_fd, SPI_IOC_MESSAGE(1), &spi);

    return rxDat;
}

void SendCommand(float v, float w) {
    uint8_t storage[8];
    float buffer[2] = {v, w};

    memcpy(storage, buffer, sizeof(buffer));

    spiTxRx(storage[0]);
    usleep(10);
    spiTxRx(storage[1]);
    usleep(10);
    spiTxRx(storage[2]);
    usleep(10);
    spiTxRx(storage[3]);
    usleep(10);
    spiTxRx(storage[4]);
    usleep(10);
    spiTxRx(storage[5]);
    usleep(10);
    spiTxRx(storage[6]);
    usleep(10);
    spiTxRx(storage[7]);
    usleep(10);

    ROS_INFO_STREAM("cmd_vel: v: " << buffer[0] << " w: " << buffer[1]);
}

void CmdVelSub(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel) {
    SendCommand(cmd_vel->twist.linear.x, cmd_vel->twist.angular.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, &CmdVelSub);

    if ((mcu_fd = open("/dev/spidev0.0", O_RDWR)) < 0) {
        ROS_FATAL("Failed to open SPI bus!");
        return 0;
    }

    // 2MHz
    unsigned int speed = 2e6;
    ioctl(mcu_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    ROS_INFO_STREAM("Connected to MCU! " << mcu_fd);

    /*
    ros::Rate rate(20);
    while (ros::ok()) {
	SendCommand(3.14, 6.28);
        rate.sleep();
    }
    */

    ros::spin();
    return 0;
}

