
#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#define MAT_ROWS 800
#define MAT_COLS 1280
#define FPN_PATH "../Samples/config/FPN_2.txt"

#include <signal.h>
#include <unistd.h>

#include "celex5_msgs/eventVector.h"

CeleX5 *pCeleX5 = new CeleX5;

using namespace std;
using namespace cv;
class SensorDataObserver : public CeleX5DataManager {
public:
  SensorDataObserver(CX5SensorDataServer *pServer) {
    m_pServer = pServer;
    m_pServer->registerData(this, CeleX5DataManager::CeleX_Frame_Data);

    data_pub = nh.advertise<celex5_msgs::eventVector>("/celex5/events", 2);

    nh.param<std::string>("celex_mode", celex_mode,
                          "Event_off_Pixel_Timestamp_Mode");
  }
  ~SensorDataObserver() {
    m_pServer->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
    delete pCeleX5;
  }

  virtual void onFrameDataUpdated(
      CeleX5ProcessedData *pSensorData); // overrides Observer operation

  CX5SensorDataServer *m_pServer;
  ros::NodeHandle nh;
  ros::Publisher data_pub;

  celex5_msgs::eventVector event_vector_msg;
  std::string celex_mode;
};

void SensorDataObserver::onFrameDataUpdated(CeleX5ProcessedData *pSensorData) {
  if (NULL == pSensorData)
    return;
  if (CeleX5::Event_Off_Pixel_Timestamp_Mode == pSensorData->getSensorMode()) {
    std::vector<EventData> vecEvent;
    celex5_msgs::event event_;

    pCeleX5->getEventDataVector(vecEvent);
    int dataSize = vecEvent.size();
    event_vector_msg.vector_length = dataSize;
    event_vector_msg.height = MAT_ROWS;
    event_vector_msg.width = MAT_COLS;
    //    prepare enevtdata
    {
      for (int i = 0; i < dataSize; i++) {
//          std::cout<<"(x,y)("<<event_.x<<","<<event_.y<<")"<<std::endl;
        event_.x = vecEvent[i].col;
        event_.y = vecEvent[i].row;
        event_.off_pixel_timestamp = vecEvent[i].tOffPixelIncreasing;
        event_.brightness = 255;
        event_vector_msg.events.push_back(event_);
      }
      data_pub.publish(event_vector_msg);
      event_vector_msg.events.clear();
    }

    // vis
    //    {
    //      cv::Mat mat = cv::Mat::zeros(cv::Size(1280, 800), CV_8UC1);
    //      for (int i = 0; i < dataSize; i++) {
    //        mat.at<uchar>(800 - vecEvent[i].row - 1, 1280 - vecEvent[i].col -
    //        1) =
    //            255;
    //      }
    //      if (dataSize > 0) {
    //        cv::imshow("Event Binary Pic", mat);
    //        cv::waitKey(1);
    //      }
    //    }

  } else {
    std::cout << "This mode has no event data. " << std::endl;
  }
}

void exit_handler(int sig_num) {
  printf("SIGNAL received: num =%d\n", sig_num);
  if (sig_num == 1 || sig_num == 2 || sig_num == 3 || sig_num == 9 ||
      sig_num == 15) {
    delete pCeleX5;
    pCeleX5 = NULL;
    exit(0);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "celex_get_event_vector");
  if (NULL == pCeleX5)
    return 0;

  pCeleX5->openSensor(CeleX5::CeleX5_MIPI);
  pCeleX5->setFpnFile(FPN_PATH);
  pCeleX5->setSensorFixedMode(CeleX5::Event_Off_Pixel_Timestamp_Mode);
  pCeleX5->disableFrameModule();
  pCeleX5->disableIMUModule();
  pCeleX5->disableEventCountSlice();
  pCeleX5->enableEventDenoising();
  SensorDataObserver *pSensorData =
      new SensorDataObserver(pCeleX5->getSensorDataServer());

  // install signal use sigaction
  struct sigaction sig_action;
  sigemptyset(&sig_action.sa_mask);
  sig_action.sa_flags = 0;
  sig_action.sa_handler = exit_handler;
  sigaction(SIGHUP, &sig_action, NULL);  // 1
  sigaction(SIGINT, &sig_action, NULL);  // 2
  sigaction(SIGQUIT, &sig_action, NULL); // 3
  sigaction(SIGKILL, &sig_action, NULL); // 9
  sigaction(SIGTERM, &sig_action, NULL); // 15

  while (true) {
#ifdef _WIN32
    Sleep(1);
#else
    usleep(1000);
#endif
  }
  return 1;
}
