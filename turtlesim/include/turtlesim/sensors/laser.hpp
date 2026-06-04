#ifndef TURTLESIM__LASER_HPP_
#define TURTLESIM__LASER_HPP_

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

#include <QLine>
#include <QPoint>
#include <QPolygon>
#include <random>

namespace turtlesim
{

class Laser
{
public:
    using MessageType = sensor_msgs::msg::LaserScan;
    explicit Laser(rclcpp::Node::SharedPtr& nodeHandle, const std::string& frame_id);

    void measure(const QPointF& position, const float orientation, const std::map<std::string, QPolygonF>& boundaries);

private:
    rclcpp::Node::SharedPtr                   nodeHandle_;
    std::string                               frame_id_;
    rclcpp::Publisher<MessageType>::SharedPtr pub_;
    std::random_device                        random_device_{};
    std::mt19937                              randomGenerator_{random_device_()};
    std::normal_distribution<float>           noise_{0.01, 0.02};
};

} // namespace turtlesim

#endif // TURTLESIM__LASER_HPP_
