/**
 * @file vicon2mavros_modify.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/bind.hpp>



const int UAV_NUM = 4;


geometry_msgs::TransformStamped current_vicon_pos[UAV_NUM + 1];

void vicon_pos_cb(const geometry_msgs::TransformStamped::ConstPtr& msg, int num){
    current_vicon_pos[num] = *msg;
}

int main(int argc, char **argv)
{
	char f330_num_str[UAV_NUM + 1][40],uav_num_str[UAV_NUM + 1][40];
	for (int i = 1; i <= UAV_NUM; i++){
		sprintf(f330_num_str[i],"/vicon/p230_%d/p230_%d",i,i);
		sprintf(uav_num_str[i],"/uav%d/mavros/vision_pose/pose",i);	
	}

    ros::init(argc, argv, "vicon2mavros");
    ros::NodeHandle nh;
	
	ros::Subscriber vicon_sub[UAV_NUM + 1];
	ros::Publisher local_pos_pub[UAV_NUM + 1];

	for (int i = 1; i <= UAV_NUM; i++){
		vicon_sub[i] = nh.subscribe<geometry_msgs::TransformStamped>(f330_num_str[i], 10, boost::bind(&vicon_pos_cb,_1, i));
    	local_pos_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(uav_num_str[i], 10);
	}

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(200.0);

    while(ros::ok()){
		
		geometry_msgs::PoseStamped pose[UAV_NUM + 1];
		for (int i = 1; i <= UAV_NUM; i++){
			pose[i].header.stamp = ros::Time::now();
			pose[i].header.frame_id = current_vicon_pos[i].header.frame_id;
			
			pose[i].pose.position.x = current_vicon_pos[i].transform.translation.x;
			pose[i].pose.position.y = current_vicon_pos[i].transform.translation.y;
			pose[i].pose.position.z = current_vicon_pos[i].transform.translation.z;

			pose[i].pose.orientation.x = current_vicon_pos[i].transform.rotation.x;
			pose[i].pose.orientation.y = current_vicon_pos[i].transform.rotation.y;
			pose[i].pose.orientation.z = current_vicon_pos[i].transform.rotation.z;
			pose[i].pose.orientation.w = current_vicon_pos[i].transform.rotation.w;

			local_pos_pub[i].publish(pose[i]);
		}
		
        printf("Sending vicon data...\n");

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
