#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <motor_control/simJointState.h>

ros::Publisher feedback_pub;
sensor_msgs::JointState feedback;

// for simulation only
ros::Publisher sim_feedback_pub;
motor_control::simJointState sim_feedback;


void cmdCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    feedback = sensor_msgs::JointState();
    for (int i=0; i < msg->position.size(); i++) {
        feedback.position.push_back(msg->position[i]);
        feedback.velocity.push_back(msg->velocity[i]);

        sim_feedback.position[i] = msg->position[i];
        sim_feedback.velocity[i] = msg->velocity[i];
    }

    //feedback_pub.publish(feedback);
    //sim_feedback_pub.publish(sim_feedback);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_echo");

    ros::NodeHandle n;

    ros::Subscriber cmd_sub = n.subscribe("/arm_control/joint_cmd", 10, cmdCallback);

    feedback_pub = n.advertise<sensor_msgs::JointState>("/arm_control/joint_telemetry", 10);
    sim_feedback_pub = n.advertise<motor_control::simJointState>("/arm_control/sim_joint_telemetry", 10);

    ros::Rate loop_rate(25);

    double default_pos[] = {1, 0, 0, 0, 0, 0, 0};
    double default_vel[] = {0, 0, 0, 0, 0, 0, 0};
    for (int i=0; i < 6; i++) {
        feedback.position.push_back(default_pos[i]);
        feedback.velocity.push_back(default_vel[i]);

        sim_feedback.position[i] = default_pos[i];
        sim_feedback.velocity[i] = default_vel[i];
    }

    while (ros::ok()) {
        feedback_pub.publish(feedback);
        sim_feedback_pub.publish(sim_feedback);
        ros::spinOnce();
        loop_rate.sleep();
    }
}