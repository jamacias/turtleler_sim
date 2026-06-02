#ifndef TURTLESIM__LASER_HPP_
#define TURTLESIM__LASER_HPP_

// This prevents a MOC error with versions of boost >= 1.48
#include <qpoint.h>
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

#include <qline.h>

namespace turtlesim
{

class Laser
{
public:
    explicit Laser(rclcpp::Node::SharedPtr& nodeHandle, const std::string& frame_id);

    void measure(const QPointF& position, const float orientation, const std::vector<QLineF>& boundaries) const;

private:
    rclcpp::Node::SharedPtr nodeHandle_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

} // namespace turtlesim

#endif // TURTLESIM__LASER_HPP_
