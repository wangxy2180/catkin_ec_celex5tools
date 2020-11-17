/*
 * Copyright (c) 2017-2020 CelePixel Technology Co. Ltd. All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>
#include <celex5/celex5processeddata.h>
#include <opencv2/opencv.hpp>
//#include <algorithm>
#include <unistd.h>
#include <ros/ros.h>

#include "celex5_msgs/eventVector.h"


#define FPN_PATH "../Samples/config/FPN_2.txt"
#define MAT_ROWS 800
#define MAT_COLS 1200
//#define BIN_FILE    "YOUR_BIN_FILE_PATH.bin"	//your bin file path
//#define BIN_FILE    "celexdata.bin"	//your bin file path
#define BIN_FILE                                                               \
  "/home/free/EventCamera/celexdata.bin"

CeleX5 *pCeleX5 = new CeleX5;

class SensorDataObserver : public CeleX5DataManager
{
public:
    SensorDataObserver(CX5SensorDataServer *pServer)
    {
        m_pServer = pServer;
        m_pServer->registerData(this, CeleX5DataManager::CeleX_Frame_Data);
        data_pub = nh.advertise<celex5_msgs::eventVector>("/celex5/readBin",2);
    }

    ~SensorDataObserver()
    {
        m_pServer->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
    }

    virtual void onFrameDataUpdated(
            CeleX5ProcessedData *pSensorData); // overrides Observer operation

    CX5SensorDataServer *m_pServer;
    celex5_msgs::eventVector event_vector_msg;
    ros::NodeHandle nh;
    ros::Publisher data_pub;
};

uint8_t *pImageBuffer = new uint8_t[CELEX5_PIXELS_NUMBER];

void SensorDataObserver::onFrameDataUpdated(CeleX5ProcessedData *pSensorData)
{
    if (NULL == pSensorData)
        return;
    CeleX5::CeleX5Mode sensorMode = pSensorData->getSensorMode();
    if (CeleX5::Full_Picture_Mode == sensorMode)
    {
        // full-frame picture
        pCeleX5->getFullPicBuffer(pImageBuffer);
        cv::Mat matFullPic(800, 1280, CV_8UC1, pImageBuffer);
        cv::imshow("FullPic", matFullPic);
        cv::waitKey(1);
    } else if (CeleX5::Event_Off_Pixel_Timestamp_Mode == sensorMode)
    {
//        std::cout << "in" << std::endl;
//        // get buffers when sensor works in EventMode
//        pCeleX5->getEventPicBuffer(pImageBuffer, CeleX5::EventBinaryPic);
//        pCeleX5->getEventPicBuffer(pImageBuffer, CeleX5::EventDenoisedBinaryPic);
//        cv::Mat matEventPic(800, 1280, CV_8UC1, pImageBuffer);
//        cv::imshow("Event Binary Pic", matEventPic);
//        cvWaitKey(1);

        std::vector<EventData> vecEvent;
        pCeleX5->getEventDataVector(vecEvent);
//        cv::Mat showit = cv::Mat::zeros(MAT_ROWS, MAT_COLS, CV_8UC1);
        int data_size = vecEvent.size();
        std::cout << "datasize:" << data_size << std::endl;

        celex5_msgs::event event_;
        event_vector_msg.vector_length = data_size;
        event_vector_msg.width = MAT_COLS;
        event_vector_msg.height = MAT_ROWS;
        for (int i = 0; i < data_size; i++) {
            event_.x = vecEvent[i].col;
            event_.y = vecEvent[i].row;
            event_.off_pixel_timestamp = vecEvent[i].tOffPixelIncreasing;
            event_.brightness = 255;
            event_vector_msg.events.push_back(event_);
        }
        data_pub.publish(event_vector_msg);
        event_vector_msg.events.clear();
//        for (int i = 0; i < data_size; i++)
//        {
//            std::cout << "()is(" << MAT_ROWS -
//                                    vecEvent[i].row - 1 << "," << MAT_COLS - vecEvent[i].col - 1 << ")" << std::endl;
//            std::cout << "(2)is(" << vecEvent[i].row << "," << vecEvent[i].col << ")" << std::endl;
//            showit.at<uchar>(MAT_ROWS - vecEvent[i].row - 1,
//                             MAT_COLS - vecEvent[i].col - 1) = 255;
//            std::cout << "curr/datasize" << i << "/" << data_size << std::endl;
//        }
//        if (data_size > 0)
//        {
//            cv::imshow("123", showit);
//            cv::waitKey(10000);
//        }
    } else if (CeleX5::Optical_Flow_Mode == sensorMode)
    {
        // full-frame optical-flow pic
        pCeleX5->getOpticalFlowPicBuffer(pImageBuffer, CeleX5::OpticalFlowPic);
        cv::Mat matOpticalFlow(800, 1280, CV_8UC1, pImageBuffer);
        cv::imshow("Optical-Flow Pic", matOpticalFlow);
        cvWaitKey(1);
    }
}

int main(int argc,char** argv)
{
    if (pCeleX5 == NULL)
        return 0;
    ros::init(argc,argv,"celex5_readBin_ros");
//    std::cout << "Please drag your celex bin file here:" << std::endl;
//    std::string file_path;
//    std::cin >> file_path;
//    file_path.erase(file_path.begin() + 0);
//    file_path.erase(file_path.end() - 1);
//
//    std::cout << "file_path is:" << file_path << std::endl;

//    bool success = pCeleX5->openBinFile(file_path); // open the bin file
    bool success = pCeleX5->openBinFile(BIN_FILE);    //open the bin file
//    pCeleX5->enableFrameDenoising();
pCeleX5->enableEventDenoising();

    CeleX5::CeleX5Mode sensorMode =
            (CeleX5::CeleX5Mode) pCeleX5->getBinFileAttributes().loopAMode;

    SensorDataObserver *pSensorData =
            new SensorDataObserver(pCeleX5->getSensorDataServer());

    while (true)
    {
        if (pCeleX5)
        {
            if (pCeleX5->readBinFileData())break; // start reading the bin file
//            std::cout << "bin file" << std::endl;
        }
        usleep(1000);
    }
    return 1;
}
