/**
 * @file publish_heartbeat.cpp
 * @author Seungwook Lee @ USRG, KAIST
 * @brief Code for testing multiple remote ROS communication.
 *        Generate hearbeat & subscribe to the reponses.
 * @version 0.1
 * @date 2023-07-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>

class PubHearbeat
{
private:
    ros::NodeHandle nh_, nhp_;
    std_msgs::Float64MultiArray tx_msg_;
    std_msgs::Float64MultiArray tx_response1_;
    std_msgs::Float64MultiArray tx_response2_;
    std_msgs::Float64MultiArray tx_response3_;

    int seq_ = 0;
    int rate_ = 10; // hz
    std::string user1_, user2_, user3_;

public:
    PubHearbeat(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~PubHearbeat();

    // Publish my hearbeat, subscribe everyone's response
    ros::Publisher tx_pub_;
    ros::Subscriber tx_response1_sub_;
    ros::Subscriber tx_response2_sub_;
    ros::Subscriber tx_response3_sub_;
    void txResponse1SubCallback(std_msgs::Float64MultiArrayConstPtr msg);
    void txResponse2SubCallback(std_msgs::Float64MultiArrayConstPtr msg);
    void txResponse3SubCallback(std_msgs::Float64MultiArrayConstPtr msg);

    void run();
    int getRate();
};

PubHearbeat::PubHearbeat(ros::NodeHandle nh, ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp)
{
    std::string ns = ros::this_node::getNamespace();
    if(ns == "/gcs")
    {
        user1_ = "gcs";
        user2_ = "usv";
        user3_ = "uav";
    }
    else if(ns == "/usv")
    {
        user1_ = "usv";
        user2_ = "gcs";
        user3_ = "uav";
    }
    else if(ns == "/uav")
    {
        user1_ = "uav";
        user2_ = "usv";
        user3_ = "gcs";
    }
    else
    {
        ROS_ERROR("CHECK NAMESPACE OPTIONS: gcs or usv or uav");
        ros::shutdown();
    }
    nhp_.param<int>("rate", rate_, 10); // hz
    tx_pub_ = nh.advertise<std_msgs::Float64MultiArray>("tx", 1);
    tx_response1_sub_ = nh.subscribe("/"+user1_+"/rx/"+user1_, 1, &PubHearbeat::txResponse1SubCallback, this);
    tx_response2_sub_ = nh.subscribe("/"+user2_+"/rx/"+user1_, 1, &PubHearbeat::txResponse2SubCallback, this);
    tx_response3_sub_ = nh.subscribe("/"+user3_+"/rx/"+user1_, 1, &PubHearbeat::txResponse3SubCallback, this);

    tx_response1_.data = std::vector<double>({-1,0});
    tx_response2_.data = std::vector<double>({-1,0});
    tx_response3_.data = std::vector<double>({-1,0});
}

PubHearbeat::~PubHearbeat()
{
}

void PubHearbeat::txResponse1SubCallback(std_msgs::Float64MultiArrayConstPtr msg)
{
    tx_response1_ = *msg;

}

void PubHearbeat::txResponse2SubCallback(std_msgs::Float64MultiArrayConstPtr msg)
{
    tx_response2_ = *msg;
}

void PubHearbeat::txResponse3SubCallback(std_msgs::Float64MultiArrayConstPtr msg)
{
    tx_response3_ = *msg;
}

int PubHearbeat::getRate()
{
    return rate_;
}

void PubHearbeat::run()
{
    tx_msg_.data.clear();
    double stamp = ((double)seq_)/((double)rate_);
    tx_msg_.data.push_back(stamp);
    tx_msg_.data.push_back(0.0);
    tx_pub_.publish(tx_msg_);

    ROS_INFO("USER: %s", user1_.c_str());
    ROS_INFO("Publishing time stamp: %.2f (%.2f sec per step)", stamp, 1.0/(double)rate_);
    ROS_INFO("%s last seen at stamp: %.2f", user1_.c_str(), tx_response1_.data.at(0));
    ROS_INFO("%s last seen at stamp: %.2f", user2_.c_str(), tx_response2_.data.at(0));
    ROS_INFO("%s last seen at stamp: %.2f", user3_.c_str(), tx_response3_.data.at(0));
    ROS_INFO("Roundtrip time from %s, %s, %s", user1_.c_str(), user2_.c_str(), user3_.c_str());
    ROS_INFO("%.2f, %.2f, %.2f", stamp - tx_response1_.data.at(0), stamp - tx_response2_.data.at(0), stamp - tx_response3_.data.at(0));
    std::cout << "\n\n" << std::endl;
    seq_ += 1;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "publish_heartbeat");
    ros::NodeHandle nh, nhp("~");
    PubHearbeat publish_heartbeat(nh, nhp);

    ros::Rate rate(publish_heartbeat.getRate());

    while(ros::ok())
    {
        publish_heartbeat.run();
        ros::spinOnce();
        rate.sleep();
    }
    // ros::spin();
    
    return 0;
}