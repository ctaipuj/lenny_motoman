#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

/*This node publishes fake joint states for the robotiq's 85 gripper to avoid warnings when launching ur3_moveit_planning_execution
*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur_gripper_fake_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    
    sensor_msgs::JointState joint_state;
    
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="robotiq_85_left_knuckle_joint";
        joint_state.position[0] = 0.0;
        joint_state.name[1] ="robotiq_85_left_finger_joint";
        joint_state.position[1] = 0.0;
        joint_state.name[2] ="robotiq_85_right_finger_joint";
        joint_state.position[2] = 0.0;

        //send the joint state
        joint_pub.publish(joint_state);
        loop_rate.sleep();
    }

    return 0;
}
