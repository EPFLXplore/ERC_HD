/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ROS2_KERBY_HARDWARE__VISIBILITY_CONTROL_H_
#define ROS2_KERBY_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROS2_KERBY_HARDWARE_EXPORT __attribute__((dllexport))
#define ROS2_KERBY_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define ROS2_KERBY_HARDWARE_EXPORT __declspec(dllexport)
#define ROS2_KERBY_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef ROS2_KERBY_HARDWARE_BUILDING_DLL
#define ROS2_KERBY_HARDWARE_PUBLIC ROS2_KERBY_HARDWARE_EXPORT
#else
#define ROS2_KERBY_HARDWARE_PUBLIC ROS2_KERBY_HARDWARE_IMPORT
#endif
#define ROS2_KERBY_HARDWARE_PUBLIC_TYPE ROS2_KERBY_HARDWARE_PUBLIC
#define ROS2_KERBY_HARDWARE_LOCAL
#else
#define ROS2_KERBY_HARDWARE_EXPORT __attribute__((visibility("default")))
#define ROS2_KERBY_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define ROS2_KERBY_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define ROS2_KERBY_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define ROS2_KERBY_HARDWARE_PUBLIC
#define ROS2_KERBY_HARDWARE_LOCAL
#endif
#define ROS2_KERBY_HARDWARE_PUBLIC_TYPE
#endif

#endif  // ROS2_KERBY_HARDWARE__VISIBILITY_CONTROL_H_