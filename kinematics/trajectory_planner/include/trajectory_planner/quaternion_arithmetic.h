#include <geometry_msgs/msg/pose.hpp>
#include "rclcpp/rclcpp.hpp"


typedef geometry_msgs::msg::Point Point;
typedef geometry_msgs::msg::Quaternion Quaternion;
typedef geometry_msgs::msg::Pose Pose;


void print(const Quaternion &q, std::string name = "") {
    RCLCPP_INFO(rclcpp::get_logger("kinematics_trajectory_planner"), "%s : w: %g, x: %g, y: %g, z: %g", name, q.w, q.x, q.y, q.z);
}

void print(const Point &p, std::string name = "") {
    RCLCPP_INFO(rclcpp::get_logger("kinematics_trajectory_planner"), "%s : x: %g, y: %g, z: %g", name, p.x, p.y, p.z);
}

Point makePoint(const Quaternion &q) {
    Point p;
    p.x = q.x;
    p.y = q.y;
    p.z = q.z;
    return p;
}

Quaternion makeQuat(const Point &p) {
    Quaternion q;
    q.w = 0.0;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    return q;
}

Quaternion mul(const Quaternion &q1, const Quaternion &q2) {
    Quaternion ans;
    ans.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    ans.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    ans.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    ans.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return ans;
}

Quaternion inv(const Quaternion &q) {
    Quaternion q_;
    q_.w = q.w;
    q_.x = -q.x;
    q_.y = -q.y;
    q_.z = -q.z;
    return q_;
}

/// calculate the image of the point p under the rotation described by the quaternion q
Point pointImage(const Point &p, const Quaternion &q) {
    Quaternion q_ = inv(q);
    Quaternion pp = makeQuat(p);
    Quaternion res = mul(mul(q, pp), q_);
    return makePoint(res);
}