#include "turtlesim/sensors/camera.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace turtlesim
{

Camera::Camera(rclcpp::Node::SharedPtr& nodeHandle)
: nodeHandle_(nodeHandle)
{
    pub_ = nodeHandle_->create_publisher<MessageType>("/bev", rclcpp::QoS(1));
}

void Camera::measure(const QImage& image)
{
    MessageType imageMsg;
    imageMsg.header.stamp    = rclcpp::Clock().now();
    imageMsg.header.frame_id = "bev_camera";
    imageMsg.height          = image.height();
    imageMsg.width           = image.width();
    imageMsg.step            = 3;
    imageMsg.encoding        = "rgb8";
    imageMsg.data.reserve(imageMsg.step * imageMsg.height * imageMsg.width);
    for (int y = 0; y < image.height(); ++y)
    {
        for (int x = 0; x < image.width(); ++x)
        {
            const auto color = image.pixelColor(x, y);
            imageMsg.data.emplace_back(color.red());
            imageMsg.data.emplace_back(color.green());
            imageMsg.data.emplace_back(color.blue());
        }
    }

    assert(imageMsg.data.size() == imageMsg.data.capacity());

    pub_->publish(imageMsg);
}

} // namespace turtlesim
