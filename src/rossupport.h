#pragma once

#include <string>

#include "ros/ros.h"
#include "gazehyps.h"
#include "gazetool/GazeHyps.h"
#include "gazetool/GazeInfo.h"
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
    bool initializationDone = false;

public:
    RosSubscriber(std::string subsTopic);
    void Callback(const sensor_msgs::ImageConstPtr& msg);

    virtual bool get(cv::Mat& frame);
    virtual std::string getLabel();
    virtual std::string getId();
    virtual ~RosSubscriber();
};

class RosPublisher {

private:
    gazetool::GazeHyps msgGaze;
    ros::Publisher gazePub;
    
    gazetool::GazeInfo msgInfo;
    ros::Publisher infoPub;

public:
    RosPublisher();
    RosPublisher(std::string rosTopicPub);
    void publishGazeHypotheses(GazeHypsPtr gazehyps);
    void publishAdditionalInformation(double horizGazeTolerance, double verticalGazeTolerance);
    ~RosPublisher();

};
