#ifndef TURTLESIM__LASER_HPP_
#define TURTLESIM__LASER_HPP_

// This prevents a MOC error with versions of boost >= 1.48
#include <qpoint.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

#include <qvector2d.h>
#include <qline.h>

namespace turtlesim
{

class Laser
{
public:
    explicit Laser(rclcpp::Node::SharedPtr& nodeHandle, const std::string& frame_id)
    : nodeHandle_(nodeHandle)
    , frame_id_(frame_id)
    {
        pub_ = nodeHandle_->create_publisher<sensor_msgs::msg::LaserScan>(frame_id_ + "/laser", rclcpp::QoS(1));
    }

    void measure(const QPointF& position, const float orientation, const std::vector<QLineF>& boundaries) const
    {
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp             = rclcpp::Clock().now();
        scan.header.frame_id          = frame_id_;
        scan.range_min                = 0.1f;
        scan.range_max                = 10.0f;
        scan.scan_time                = 0.01f;
        scan.time_increment           = 0.001f;
        scan.angle_min                = -2 * M_PI / 3;
        scan.angle_max                = 2 * M_PI / 3;
        constexpr std::size_t n_beams = 100;
        scan.angle_increment          = (scan.angle_max - scan.angle_min) / n_beams;
        for (std::size_t i = 0; i < n_beams; ++i)
        {
            // Determine origin and direction (ray)
            const auto beam_orientation = orientation + scan.angle_min + i * scan.angle_increment;
            const auto origin           = QVector2D(position);
            const auto direction = QVector2D(std::cos(beam_orientation), -std::sin(beam_orientation)).normalized();
            const auto ray = QLineF(origin.toPointF(), (1000.0 * direction).toPointF());
            // auto ray = QLineF(origin.toPointF(), QVector2D(100.0, 100.0).toPointF());
            // ray.setAngle(beam_orientation * 180.0 / M_PI);
            // RCLCPP_WARN(nh_->get_logger(), "Ray p1 [x, y]: [%.3f, %.3f]; ray p2: [%.3f, %.3f]", ray.p1().x(), ray.p1().y(), ray.p2().x(), ray.p2().y());
            // Find the intersection with the lines defining the boundaries
            float range = std::numeric_limits<float>::infinity();
            for (const auto& boundary : boundaries)
            {
                QPointF intersection;
                if (const auto intersectionType = ray.intersects(boundary, &intersection);
                    intersectionType == QLineF::IntersectionType::BoundedIntersection)
                {
                    if (const auto distance = origin.distanceToPoint(QVector2D(intersection)); range > distance)
                    {
                        range = distance;
                    }
                }
            }
            scan.ranges.emplace_back(range);
        }

        pub_->publish(scan);
    }

private:
    rclcpp::Node::SharedPtr nodeHandle_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

} // namespace turtlesim

#endif // TURTLESIM__LASER_HPP_
