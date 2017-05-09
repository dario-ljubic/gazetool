#pragma once

#include <string>

#include "ros/ros.h"
#include "gazehyps.h"
#include "gaze_test/GazeHyps.h"
#include "std_msgs/String.h"

#include "imageprovider.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class RosSubscriber : public ImageProvider
{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    cv_bridge::CvImagePtr cvImagePtr;

public:
    RosSubscriber();
    void Callback(const sensor_msgs::ImageConstPtr& msg);

    virtual bool get(cv::Mat& frame);
    virtual std::string getLabel();
    virtual std::string getId();
    virtual ~RosSubscriber();
};

class RosPublisher {

private:
    gaze_test::GazeHyps msg;
    ros::Publisher pub;

public:
    RosPublisher();
    RosPublisher(std::string rosTopicPub);
    void publishGazeHypotheses(GazeHypsPtr gazehyps);
    ~RosPublisher();

};
