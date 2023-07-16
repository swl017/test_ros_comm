/**
 * @file respond_to_heartbeats.cpp
 * @author Seungwook Lee @ USRG, KAIST
 * @brief Code for testing multiple remote ROS communication.
 *        Subscribe and give response
 * @version 0.1
 * @date 2023-07-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>


class RespondToHeartbeats
{
private:
    /* data */
    ros::NodeHandle nh_, nhp_;
    std::string user1_, user2_, user3_;
    std_msgs::Float64MultiArray tx1_, tx2_, tx3_;

public:
    RespondToHeartbeats(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~RespondToHeartbeats();

    // Subscribe to everyone's hearbeat(rx), publish my reponse
    ros::Subscriber tx1_sub_;
    ros::Subscriber tx2_sub_;
    ros::Subscriber tx3_sub_;
    void tx1SubCallback(std_msgs::Float64MultiArrayConstPtr msg);
    void tx2SubCallback(std_msgs::Float64MultiArrayConstPtr msg);
    void tx3SubCallback(std_msgs::Float64MultiArrayConstPtr msg);
    ros::Publisher tx1_response_pub_;
    ros::Publisher tx2_response_pub_;
    ros::Publisher tx3_response_pub_;

    void printInfo();
};

RespondToHeartbeats::RespondToHeartbeats(ros::NodeHandle nh, ros::NodeHandle nhp)
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

    tx1_sub_ = nh.subscribe("/"+user1_+"/tx", 1, &RespondToHeartbeats::tx1SubCallback, this);
    tx1_response_pub_ = nh.advertise<std_msgs::Float64MultiArray>("rx/"+user1_, 1);
    tx2_sub_ = nh.subscribe("/"+user2_+"/tx", 1, &RespondToHeartbeats::tx2SubCallback, this);
    tx2_response_pub_ = nh.advertise<std_msgs::Float64MultiArray>("rx/"+user2_, 1);
    tx3_sub_ = nh.subscribe("/"+user3_+"/tx", 1, &RespondToHeartbeats::tx3SubCallback, this);
    tx3_response_pub_ = nh.advertise<std_msgs::Float64MultiArray>("rx/"+user3_, 1);

}

RespondToHeartbeats::~RespondToHeartbeats()
{
}

void RespondToHeartbeats::tx1SubCallback(std_msgs::Float64MultiArrayConstPtr msg)
{
    tx1_ = *msg;
    tx1_response_pub_.publish(*msg);
    double stamp = msg->data.size() > 0 ? msg->data.at(0) : -1;
    // ROS_INFO("Responding to %s, stamp %.2f \n", user1_.c_str(), stamp);
    printInfo();
}

void RespondToHeartbeats::tx2SubCallback(std_msgs::Float64MultiArrayConstPtr msg)
{
    tx2_ = *msg;
    tx2_response_pub_.publish(*msg);
    double stamp = msg->data.size() > 0 ? msg->data.at(0) : -1;
    // ROS_INFO("Responding to %s, stamp %.2f \n", user2_.c_str(), stamp);
    printInfo();
}

void RespondToHeartbeats::tx3SubCallback(std_msgs::Float64MultiArrayConstPtr msg)
{
    tx3_ = *msg;
    tx3_response_pub_.publish(*msg);
    double stamp = msg->data.size() > 0 ? msg->data.at(0) : -1;
    // ROS_INFO("Responding to %s, stamp %.2f \n", user3_.c_str(), stamp);
    printInfo();
}

void RespondToHeartbeats::printInfo()
{
    double stamp1 = tx1_.data.size() > 0 ? tx1_.data.at(0) : -1;
    double stamp2 = tx2_.data.size() > 0 ? tx2_.data.at(0) : -1;
    double stamp3 = tx3_.data.size() > 0 ? tx3_.data.at(0) : -1;
    ROS_INFO("Responding to %s, %s, %s stamp %.2f, %.2f, %.2f \n", user1_.c_str(), user2_.c_str(), user3_.c_str(), stamp1, stamp2, stamp3);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "respond_to_heartbeats");
    ros::NodeHandle nh, nhp("~");
    RespondToHeartbeats respond_to_heartbeats(nh, nhp);

    // ros::Rate rate(respond_to_heartbeats.getRate());

    // while(ros::ok())
    // {
    //     respond_to_heartbeats.run();
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    ros::spin();
    
    return 0;
}