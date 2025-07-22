#ifndef kinova_interface_HPP_
#define kinova_interface_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "kinova_interface/visibility_control.h"

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <KError.h>

namespace k_api = Kinova::Api;

using namespace geometry_msgs::msg;

constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

namespace kinova_interface
{

    class KinovaInterfaceNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the KinovaInterfaceNode
         */
        explicit KinovaInterfaceNode();

        /**
         * @brief Destructor for the KinovaInterfaceNode
         */
        virtual ~KinovaInterfaceNode() = default;

    private:
        rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
        k_api::Base::BaseClient *base_;
        k_api::BaseCyclic::BaseCyclicClient *base_cyclic_;

        k_api::Base::Twist *k_api_twist_;
        k_api::Base::TwistCommand k_api_twist_command_;

        Twist::SharedPtr target_twist_;
        rclcpp::Time last_twist_time_;
        std::mutex twist_mutex_;
        rclcpp::TimerBase::SharedPtr controller_timer_;

        bool move_to_home(k_api::Base::BaseClient *base);
        bool example_cartesian_action_movement(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic);
        void twist_cb(const Twist::SharedPtr msg);
        void spin_controller();
        std::function<void(k_api::Base::ActionNotification)> create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise);
    };

} // namespace grid_generation

#endif // kinova_interface_HPP_