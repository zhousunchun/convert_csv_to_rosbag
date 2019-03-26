//
// Created by chunchun on 19-3-26.
//


#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <iostream>
#include "houzhan/csv.h"

int main(int argc, char * argv[])
{
    if(argc !=3)
    {
        std::cout << "you do not have enough paremeter \n Basic usage: ./exec csv_file_path where_to_save_rosbag " << std::endl;
        return -1;
    }

    std::string pathToFile = argv[1];
    std::string pathToSaveRosBag = argv[2];
    int convertFrequency = 0;
    ros::init(argc,argv,"convert_csv_to_rosbag");
    ros::NodeHandle nh;
    rosbag::Bag bag;

    bag.open(pathToSaveRosBag,rosbag::bagmode::Write);


    io::CSVReader<38> in(pathToFile);
    in.read_header(io::ignore_extra_column, "%field.header.stamp","field.orientation.x","field.orientation.y","field.orientation.z","field.orientation.w",
            "field.orientation_covariance0", "field.orientation_covariance1", "field.orientation_covariance2",
            "field.orientation_covariance3", "field.orientation_covariance4", "field.orientation_covariance5",
            "field.orientation_covariance6", "field.orientation_covariance7", "field.orientation_covariance8",
            "field.angular_velocity.x", "field.angular_velocity.y", "field.angular_velocity.z",
            "field.angular_velocity_covariance0","field.angular_velocity_covariance1","field.angular_velocity_covariance2",
            "field.angular_velocity_covariance3","field.angular_velocity_covariance4","field.angular_velocity_covariance5",
            "field.angular_velocity_covariance6","field.angular_velocity_covariance7","field.angular_velocity_covariance8",
            "field.linear_acceleration.x","field.linear_acceleration.y","field.linear_acceleration.z",
            "field.linear_acceleration_covariance0","field.linear_acceleration_covariance1","field.linear_acceleration_covariance2",
            "field.linear_acceleration_covariance3","field.linear_acceleration_covariance4","field.linear_acceleration_covariance5",
            "field.linear_acceleration_covariance6","field.linear_acceleration_covariance7","field.linear_acceleration_covariance8"
            );
    int64_t timestamp;
    double orientation[4];
    double orientationCovariance[9];
    double angularVelocity[3];
    double angularVelocityCovariance[9];
    double acceleration[3];
    double accelerationCovariance[9];

    std::cout << "This program start to convert your specified csv file to rosbag \nPlease wait a seconds ..." << std::endl;
    while(in.read_row(timestamp, orientation[0],orientation[1],orientation[2],orientation[3]
    ,orientationCovariance[0],orientationCovariance[1],orientationCovariance[2]
    ,orientationCovariance[3],orientationCovariance[4],orientationCovariance[5]
    ,orientationCovariance[6],orientationCovariance[7],orientationCovariance[8]
    ,angularVelocity[0],angularVelocity[1],angularVelocity[2]
    ,angularVelocityCovariance[0],angularVelocityCovariance[1],angularVelocityCovariance[2]
    ,angularVelocityCovariance[3],angularVelocityCovariance[4],angularVelocityCovariance[5]
    ,angularVelocityCovariance[6],angularVelocityCovariance[7],angularVelocityCovariance[8]
    ,acceleration[0],acceleration[1],acceleration[2]
    ,accelerationCovariance[0],accelerationCovariance[1],accelerationCovariance[2]
    ,accelerationCovariance[3],accelerationCovariance[4],accelerationCovariance[5]
    ,accelerationCovariance[6],accelerationCovariance[7],accelerationCovariance[8]))
    {
        sensor_msgs::Imu imu;
        imu.header.frame_id = "imu";
        imu.orientation.x = orientation[0];
        imu.orientation.y = orientation[1];
        imu.orientation.z = orientation[2];
        imu.orientation.w = orientation[3];
        imu.orientation_covariance[0] = orientationCovariance[0];
        imu.orientation_covariance[1] = orientationCovariance[1];
        imu.orientation_covariance[2] = orientationCovariance[2];
        imu.orientation_covariance[3] = orientationCovariance[3];
        imu.orientation_covariance[4] = orientationCovariance[4];
        imu.orientation_covariance[5] = orientationCovariance[5];
        imu.orientation_covariance[6] = orientationCovariance[6];
        imu.orientation_covariance[7] = orientationCovariance[7];
        imu.orientation_covariance[8] = orientationCovariance[8];

        imu.angular_velocity.x = angularVelocity[0];
        imu.angular_velocity.y = angularVelocity[1];
        imu.angular_velocity.z = angularVelocity[2];

        imu.angular_velocity_covariance[0] = angularVelocityCovariance[0];
        imu.angular_velocity_covariance[1] = angularVelocityCovariance[1];
        imu.angular_velocity_covariance[2] = angularVelocityCovariance[2];
        imu.angular_velocity_covariance[3] = angularVelocityCovariance[3];
        imu.angular_velocity_covariance[4] = angularVelocityCovariance[4];
        imu.angular_velocity_covariance[5] = angularVelocityCovariance[5];
        imu.angular_velocity_covariance[6] = angularVelocityCovariance[6];
        imu.angular_velocity_covariance[7] = angularVelocityCovariance[7];
        imu.angular_velocity_covariance[8] = angularVelocityCovariance[8];

        imu.linear_acceleration.x = acceleration[0];
        imu.linear_acceleration.y = acceleration[1];
        imu.linear_acceleration.z = acceleration[2];

        bag.write("/imu", ros::Time::now(), imu);

    }

    bag.close();
    std::cout << "The program finished "<< std::endl;
    return 0;
}