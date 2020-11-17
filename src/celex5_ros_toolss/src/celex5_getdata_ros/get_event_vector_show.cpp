//
// Created by free on 2020/11/14.
//
#include "celex5_msgs/eventVector.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <iostream>

#define MAT_COLS 1280
#define MAT_ROWS 800

void show_callback(const celex5_msgs::eventVector::ConstPtr &msg)
{
    ROS_INFO("I heard celex5 data size: [%d]", msg->vector_length);

    // display the image
    if (msg->vector_length > 0)
    {
        cv::Mat mat = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);

        std::cout<<"======length"<<msg->vector_length<<std::endl;
//    cv::Mat mat = cv::Mat::zeros(cv::Size(MAT_ROWS, MAT_COLS), CV_8UC1);
        for (int i = 0; i < msg->vector_length; i++)
        {
//            std::cout<<"matsize(r,c)("<<mat.rows<<","<<mat.cols<<")"<<std::endl;
//            std::cout << "x,y(" << msg->events[i].x << "," << msg->events[i].y << ")" << std::endl;
//            std::cout << "row,col(" << MAT_ROWS - msg->events[i].y - 1 << ","
//                      << MAT_COLS - msg->events[i].x - 1 << ")"
//                      << std::endl;
            mat.at<uchar>(MAT_ROWS - msg->events[i].y - 1,
                          MAT_COLS - msg->events[i].x - 1) =
                    msg->events[i].brightness;
// for some sb!
//      mat.at<uchar>(MAT_ROWS - msg->events[i].x - 1,
//                    MAT_COLS - msg->events[i].y - 1) =
//          msg->events[i].brightness;
        }
        cv::imshow("event_vector_show", mat);
        cv::waitKey(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "event_vector_show");
    ros::NodeHandle nh;
    ros::Subscriber event_vec_sub;
    std::string get_data_topic;
//    get_data_topic = "/celex5/readBin";
//    get_data_topic = "/celex5/events";
//    get_data_topic = "/feature_events";
//    get_data_topic = "/celex5_mipi/events";
    get_data_topic = "/input_events";

    event_vec_sub = nh.subscribe(get_data_topic, 1, show_callback);
//    event_vec_sub = nh.subscribe("/input_events", 1, show_callback);

    ros::spin();
}