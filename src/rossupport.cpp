#include "rossupport.h"

RosSubscriber::RosSubscriber(std::string subsTopic) : it(nh)
{
    sub = it.subscribe(subsTopic, 1, &RosSubscriber::Callback, this);
}

void RosSubscriber::Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cvImagePtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

bool RosSubscriber::get(cv::Mat& frame){
    //todo:add some conditions if the topic is not created etc. (like in yarpsupport)
    // na pocetku, vrti ga vise puta dok ne dobijem pointer. Znaci samo u while pukni spinOnce pa kad se zalaufa onda ce biti dobro dalje
    ros::spinOnce();
    //if (cvImagePtr) ROS_ERROR("cvImagePtr!");
    if (cvImagePtr && ros::ok()) { // TODO: test if this causes any problems with running gazetool (cvImagePtr && ros::ok())
        cv::Mat rgbframe(cvImagePtr->image);
        frame = rgbframe.clone();
        return true;
    }
    ROS_ERROR("No input topic provided!");
    return false;
}

std::string RosSubscriber::getLabel()
{
    return "";
}

std::string RosSubscriber::getId()
{
    return "";
}

RosSubscriber::~RosSubscriber(){

}


RosPublisher::RosPublisher(){
}

RosPublisher::RosPublisher(std::string rosTopicPub) {
    ros::NodeHandle nh;
    gazePub = nh.advertise<gazetool::GazeHyps>(rosTopicPub, 50); // under assumption that the gazetool is taking ~20 fps, hold the last 2.5 sec
    infoPub = nh.advertise<gazetool::GazeInfo>("additionalGazetoolInformation", 50);
    
}

void RosPublisher::publishAdditionalInformation(double horizGazeTolerance, double verticalGazeTolerance)
{
    msgInfo.horizontalGazeTolerance = horizGazeTolerance;
    msgInfo.verticalGazeTolerance = verticalGazeTolerance;
    
    infoPub.publish(msgInfo);
}


void RosPublisher::publishGazeHypotheses(GazeHypsPtr gazehyps) {

//    msgGaze.header.seq = ; //uint32
//    msgGaze.header.stamp = ; //time
//    msgGaze.header.frame_id = ; //string

    double lid = std::nan("not set"); //todo: find a default number instead of nan!
    double horest = std::nan("not set");
    double vertest = std::nan("not set");
    bool mutgaze = false;
    if (gazehyps->size()) {
        GazeHyp& ghyp = gazehyps->hyps(0);
        lid = ghyp.eyeLidClassification.get_value_or(lid);
        horest = ghyp.horizontalGazeEstimation.get_value_or(horest);
        vertest = ghyp.verticalGazeEstimation.get_value_or(vertest);
        mutgaze = ghyp.isMutualGaze.get_value_or(false);
    }
    msgGaze.frame = gazehyps->frameCounter;
//    msgGaze.id = gazehyps->id;
//    msgGaze.label = gazehyps->label;
//    msgGaze.lid = lid;
    msgGaze.horGaze = horest;
    msgGaze.verGaze = vertest;
    msgGaze.mutGaze = mutgaze;

    gazePub.publish(msgGaze);
    //ros::spinOnce();
}

RosPublisher::~RosPublisher(){

}
