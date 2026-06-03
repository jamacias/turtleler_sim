#include "turtlesim/sensors/laser.hpp"

#include <qvector2d.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace turtlesim
{

Laser::Laser(rclcpp::Node::SharedPtr& nodeHandle, const std::string& frame_id)
: nodeHandle_(nodeHandle)
, frame_id_(frame_id)
{
    pub_ = nodeHandle_->create_publisher<MessageType>(frame_id_ + "/laser", rclcpp::QoS(1));
}

void Laser::measure(const QPointF& position, const float orientation, const std::vector<QLineF>& boundaries)
{
    MessageType scan;
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
        // Make a ray going from the turtle at the angle of the beam in world
        const auto beam_orientation = orientation + scan.angle_min + i * scan.angle_increment;
        auto       ray              = QLineF(position, QPointF(position.x() + scan.range_max, 0.0f));
        ray.setAngle(beam_orientation * 180.0 / M_PI);

        // The range is the closest point that intersects with the boundary lines
        float range = std::numeric_limits<float>::infinity();
        for (const auto& boundary : boundaries)
        {
            QPointF intersection;
            if (const auto intersectionType = ray.intersects(boundary, &intersection);
                intersectionType != QLineF::IntersectionType::BoundedIntersection)
            {
                continue;
            }

            if (const auto distance = QVector2D(position).distanceToPoint(QVector2D(intersection)) + noise_(randomGenerator_);
                range > distance)
            {
                range = distance;
            }
        }
        scan.ranges.emplace_back(range);
    }

    pub_->publish(scan);
}

} // namespace turtlesim
