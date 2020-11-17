//
// Created by free on 2020/11/11.
//
#include "detector.h"

namespace corner_detector
{
    Detector::Detector()
    {
        feature_pub = nh.advertise<celex5_msgs::eventVector>("/feature_events", 1);
        event_sub = nh.subscribe("/celex5/events", 0, &Detector::eventCallback, this);
    }

    Detector::~Detector()
    {}

/*
 * call back only do one thing for all detector
 * search feature corner and do some statistics
 */
    void Detector::eventCallback(const celex5_msgs::eventVector::ConstPtr &msg)
    {
        celex5_msgs::eventVector feature_msg;
        feature_msg.header=msg->header;
        feature_msg.width = msg->width;
        feature_msg.height = msg->height;
//        feature_msg.vector_length=msg->vector_length;
//        feature_msg.vectorIndex = msg->vectorIndex;

        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        std::chrono::time_point<std::chrono::high_resolution_clock> cur_time;
        start_time = std::chrono::high_resolution_clock::now();
        //  traverse(bian li)all events, if it meet the condition(isFeature), pushback
        for (const auto e : msg->events)
        {
//            std::chrono::time_point<std::chrono::high_resolution_clock> feature_time;
//            std::chrono::time_point<std::chrono::high_resolution_clock> feature_end_time;
//            feature_time=std::chrono::high_resolution_clock::now();
            if (isFeature(e))
            {
                feature_msg.events.push_back(e);
            }
//            feature_end_time=std::chrono::high_resolution_clock::now();
//            const auto detect_time = std::chrono::duration_cast<std::chrono::nanoseconds>(feature_end_time - feature_time).count();
//            std::cout<<"one event time is"<<detect_time<<std::endl;
        }
        cur_time = std::chrono::high_resolution_clock::now();
        //const double elapsed = 0.0;
        // get ns level elapsed time, maybe divided by 1000 or 1000000 to get mirco or
        // milli seconds
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(cur_time - start_time).count();
        std::cout << "elapsed is " << elapsed << "ns" << std::endl;

        //  publish the feature point
        feature_msg.vector_length=feature_msg.events.size();
        feature_pub.publish(feature_msg);

        //  statistics
        //  some events happend
        const int num_events = msg->events.size();
        if (num_events > 0)
        {
            const int num_features = feature_msg.events.size();
            // so how to translate the next two, what is it meaning
            const float reducation_rate =
                    100. * (1. - num_features / (float) num_events);
            const float reducation_factor = num_events / (float) num_features;
            const float events_per_second = float(num_events) / (elapsed / 1e9);
            const float ns_per_event = elapsed / float(num_events);

            ROS_INFO("Fast detector,events: %d; features: %d; reduction rate:%.3f%%(%0.f x); speed %.0f events/s , %.0f ns/event",
                     num_events, num_features,reducation_rate,reducation_factor, events_per_second, ns_per_event);
        } else
        {
            ROS_INFO("no event happend");
        }
    }

} // namespace corner_detector