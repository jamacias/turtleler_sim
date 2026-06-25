#ifndef TURTLESIM__CAMERA_HPP_
#define TURTLESIM__CAMERA_HPP_

// This prevents a MOC error with versions of boost >= 1.48
#include "sensor_msgs/msg/image.hpp"
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

#include <QImage>
#include <random>

namespace turtlesim
{

class Camera
{
public:
    using MessageType = sensor_msgs::msg::Image;
    explicit Camera(rclcpp::Node::SharedPtr& nodeHandle);

    void measure(const QImage &image);

private:
    rclcpp::Node::SharedPtr                   nodeHandle_;
    rclcpp::Publisher<MessageType>::SharedPtr pub_;
    std::random_device                        random_device_{};
    std::mt19937                              randomGenerator_{random_device_()};
    std::normal_distribution<float>           noise_{0.01, 0.02};
};

} // namespace turtlesim

#endif // TURTLESIM__CAMERA_HPP_
