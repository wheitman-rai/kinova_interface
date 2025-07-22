#include "kinova_interface/kinova_interface.hpp"

namespace kinova_interface
{

    KinovaInterfaceNode::KinovaInterfaceNode()
        : Node("kinova_interface")
    {
        RCLCPP_INFO(this->get_logger(), "Kinova Teleop Node Started");

        // Create API objects
        auto error_callback = [](k_api::KError err)
        { std::cout << "_________ callback error _________" << err.toString(); };
        auto transport = new k_api::TransportClientTcp();
        auto router = new k_api::RouterClient(transport, error_callback);
        RCLCPP_INFO(this->get_logger(), "Connecting...");

        transport->connect("192.168.1.10", 10000);

        RCLCPP_INFO(this->get_logger(), "Connected");

        // Set session data connection information
        auto create_session_info = k_api::Session::CreateSessionInfo();
        create_session_info.set_username("admin");
        create_session_info.set_password("admin");
        create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
        create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)
        std::cout << "Creating session for communication" << std::endl;
        auto session_manager = new k_api::SessionManager(router);
        session_manager->CreateSession(create_session_info);
        std::cout << "Session created" << std::endl;

        // Create services
        base_ = new k_api::Base::BaseClient(router);
        base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router);

        // Example core
        bool success = true;
        success &= move_to_home(base_);
        // success &= example_cartesian_action_movement(base, base_cyclic);

        // Subscribers
        twist_sub_ = create_subscription<Twist>(
            "/cmd_vel", 10,
            std::bind(&KinovaInterfaceNode::twist_cb, this, std::placeholders::_1));

        // Timers: 100Hz = 10ms
        controller_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&KinovaInterfaceNode::spin_controller, this));

        // initialize kortex api twist commandd
        {
            k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
            // command.set_duration = execute time (milliseconds) according to the api ->
            // (not implemented yet)
            // see: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/TwistCommand.md
            k_api_twist_command_.set_duration(0);
            k_api_twist_ = k_api_twist_command_.mutable_twist();
        }

        RCLCPP_INFO(get_logger(), "Listening for Twist messages.");
    }

    bool KinovaInterfaceNode::move_to_home(k_api::Base::BaseClient *base)
    {
        // Make sure the arm is in Single Level Servoing before executing an Action
        auto servoingMode = k_api::Base::ServoingModeInformation();
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Move arm to ready position
        std::cout << "Moving the arm to a safe position" << std::endl;
        auto action_type = k_api::Base::RequestedActionType();
        action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
        auto action_list = base->ReadAllActions(action_type);
        auto action_handle = k_api::Base::ActionHandle();
        action_handle.set_identifier(0);
        for (auto action : action_list.action_list())
        {
            if (action.name() == "Home")
            {
                action_handle = action.handle();
            }
        }

        if (action_handle.identifier() == 0)
        {
            std::cout << "Can't reach safe position, exiting" << std::endl;
            return false;
        }
        else
        {
            // Connect to notification action topic
            std::promise<k_api::Base::ActionEvent> finish_promise;
            auto finish_future = finish_promise.get_future();
            auto promise_notification_handle = base->OnNotificationActionTopic(
                create_event_listener_by_promise(finish_promise),
                k_api::Common::NotificationOptions());

            // Execute action
            base->ExecuteActionFromReference(action_handle);

            // Wait for future value from promise
            const auto status = finish_future.wait_for(TIMEOUT_DURATION);
            base->Unsubscribe(promise_notification_handle);

            if (status != std::future_status::ready)
            {
                std::cout << "Timeout on action notification wait" << std::endl;
                return false;
            }
            const auto promise_event = finish_future.get();

            std::cout << "Move to Home completed" << std::endl;
            std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

            return true;
        }
    }

    void KinovaInterfaceNode::twist_cb(const Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        target_twist_ = msg;
        last_twist_time_ = this->now();
    }

    void KinovaInterfaceNode::spin_controller()
    {
        // Control loop at 100Hz, use latest twist if not stale
        if (base_cyclic_ == nullptr || base_ == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "base_ or base_cyclic_ is nullptr.");
            return;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Sending twist to robot");
        }

        Twist::SharedPtr twist_copy;
        rclcpp::Time twist_time;
        {
            std::lock_guard<std::mutex> lock(twist_mutex_);
            twist_copy = target_twist_;
            twist_time = last_twist_time_;
        }

        if (!twist_copy)
        {
            // No twist received yet
            return;
        }

        // Check staleness (0.1s = 100ms)
        auto now = this->now();
        if ((now - twist_time).seconds() > 0.1)
        {
            // Stale, ignore
            RCLCPP_WARN(get_logger(), "Stale, ignoring");
            return;
        }

        // Use twist_copy for control
        auto feedback = base_cyclic_->RefreshFeedback();
        auto action = k_api::Base::Action();
        action.set_name("Example Cartesian action movement");
        action.set_application_data("");

        auto constrained_pose = action.mutable_reach_pose();
        auto pose = constrained_pose->mutable_target_pose();
        pose->set_x(feedback.base().tool_pose_x() + twist_copy->linear.x * 0.01); // x (meters)
        pose->set_y(feedback.base().tool_pose_y() + twist_copy->linear.y * 0.01); // y (meters)
        pose->set_z(feedback.base().tool_pose_z() + twist_copy->linear.z * 0.01); // z (meters)
        pose->set_theta_x(feedback.base().tool_pose_theta_x());                   // theta x (degrees)
        pose->set_theta_y(feedback.base().tool_pose_theta_y());                   // theta y (degrees)
        pose->set_theta_z(feedback.base().tool_pose_theta_z());                   // theta z (degrees)

        k_api_twist_->set_linear_x(static_cast<float>(twist_copy->linear.x));
        k_api_twist_->set_linear_y(static_cast<float>(twist_copy->linear.y));
        k_api_twist_->set_linear_z(static_cast<float>(twist_copy->linear.z));
        k_api_twist_->set_angular_x(static_cast<float>(twist_copy->angular.x));
        k_api_twist_->set_angular_y(static_cast<float>(twist_copy->angular.y));
        k_api_twist_->set_angular_z(static_cast<float>(twist_copy->angular.z));
        base_->SendTwistCommand(k_api_twist_command_);

        // std::promise<k_api::Base::ActionEvent> finish_promise;
        // auto finish_future = finish_promise.get_future();
        // auto promise_notification_handle = base_->OnNotificationActionTopic(
        //     create_event_listener_by_promise(finish_promise),
        //     k_api::Common::NotificationOptions());

        // base_->ExecuteAction(action);

        // const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        // base_->Unsubscribe(promise_notification_handle);

        // if (status != std::future_status::ready)
        // {
        //     RCLCPP_ERROR(get_logger(), "Cartesian movement action timed out");
        // }

        // const auto promise_event = finish_future.get();
        // RCLCPP_INFO_STREAM(get_logger(), "Promise value: " << k_api::Base::ActionEvent_Name(promise_event));
    }

    bool KinovaInterfaceNode::example_cartesian_action_movement(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic)
    {
        std::cout << "Starting Cartesian action movement ..." << std::endl;

        auto feedback = base_cyclic->RefreshFeedback();
        auto action = k_api::Base::Action();
        action.set_name("Example Cartesian action movement");
        action.set_application_data("");

        auto constrained_pose = action.mutable_reach_pose();
        auto pose = constrained_pose->mutable_target_pose();
        pose->set_x(feedback.base().tool_pose_x());             // x (meters)
        pose->set_y(feedback.base().tool_pose_y() - 0.1);       // y (meters)
        pose->set_z(feedback.base().tool_pose_z() - 0.2);       // z (meters)
        pose->set_theta_x(feedback.base().tool_pose_theta_x()); // theta x (degrees)
        pose->set_theta_y(feedback.base().tool_pose_theta_y()); // theta y (degrees)
        pose->set_theta_z(feedback.base().tool_pose_theta_z()); // theta z (degrees)

        // Connect to notification action topic
        // (Reference alternative)
        // See angular examples for Promise alternative

        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions());

        std::cout << "Executing action" << std::endl;
        base->ExecuteAction(action);

        std::cout << "Waiting for movement to finish ..." << std::endl;

        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if (status != std::future_status::ready)
        {
            RCLCPP_INFO(get_logger(), "Cartesian movement action timed out");
            return false;
        }

        const auto promise_event = finish_future.get();

        RCLCPP_INFO(get_logger(), "Cartesian movement complete");
        RCLCPP_INFO(get_logger(), "Promise value: ", k_api::Base::ActionEvent_Name(promise_event));

        return true;
    }

    std::function<void(k_api::Base::ActionNotification)> KinovaInterfaceNode::create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise)
    {
        return [&finish_promise](k_api::Base::ActionNotification notification)
        {
            const auto action_event = notification.action_event();
            switch (action_event)
            {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
            }
        };
    }
}