#include "get_pose.h"
#include "std_msgs/Bool.h"

namespace dc
{
static std::string frame = "base_link"; // The tf we work in
}

int main(int argc, char** argv) {

    // Set up ROS stuff
    ros::init(argc, argv, "get_pose");
    ros::AsyncSpinner spinner(1);
    spinner.start();     

    dataCollection dc = dataCollection();
     
    ROS_INFO_STREAM("Shutting down.");
    ros::shutdown();
    return 0;
};


dataCollection::dataCollection() :
    node_(),
    frame_("/base_link"),
    moveInterface_()    
{
    tf::TransformListener tfListener;
    moveInterface_.initialize("sia5");
    moveInterface_.setPlannerId("RRTConnectkConfigDefault");
    moveInterface_.setPlanningTime(10.0);

   
    geometry_msgs::PoseStamped pose;
    pose = moveInterface_.getCurrentPose();
    tfListener.waitForTransform(pose.header.frame_id.c_str(), frame_, ros::Time::now(), ros::Duration(10.0));
    tfListener.transformPose(frame_, pose, pose);
/*    
ROS_INFO_STREAM(pose); 
*/    
   
    

    std::vector<double> joints;
    joints = moveInterface_.getCurrentJointValues();
    ROS_INFO_STREAM("Joint_0=" << joints[0] << "Joint_1=" << joints[1] << "Joint_2=" << joints[2] << "Joint_3=" << joints[3] << "Joint_4=" << joints[4]<< "Joint_5=" << joints[5]<< "Joint_6=" << joints[6]);

 
}
