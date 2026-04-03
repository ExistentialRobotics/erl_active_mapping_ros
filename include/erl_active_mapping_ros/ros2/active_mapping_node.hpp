#pragma once

#include "erl_common/ros2_topic_params.hpp"
#include "erl_common/yaml.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <memory>

inline rclcpp::Node *g_active_mapping_node = nullptr;

struct ActiveMappingNodeConfig : public erl::common::Yamlable<ActiveMappingNodeConfig> {
    using Ros2TopicParams = erl::common::ros_params::Ros2TopicParams;

    std::string global_frame = "map";
    std::string robot_frame = "base_link";
    bool auto_replan = false;
    double stop_exploration_ratio = 0.95;
    Ros2TopicParams path_topic{"path"};
    Ros2TopicParams goal_topic{"goal"};
    Ros2TopicParams dist_topic{"distance"};
    Ros2TopicParams observed_area_topic{"observed_area"};
    Ros2TopicParams observed_ratio_topic{"observed_ratio"};
    Ros2TopicParams replan_topic{"replan"};
    Ros2TopicParams plan_srv{"plan_path", "services"};
    Ros2TopicParams enable_srv{"enable", "services"};
    double max_observed_area = 10.0;

    ERL_REFLECT_SCHEMA(
        ActiveMappingNodeConfig,
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, global_frame),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, robot_frame),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, auto_replan),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, stop_exploration_ratio),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, path_topic),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, goal_topic),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, dist_topic),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, observed_area_topic),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, observed_ratio_topic),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, replan_topic),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, plan_srv),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, enable_srv),
        ERL_REFLECT_MEMBER(ActiveMappingNodeConfig, max_observed_area));

    bool
    PostDeserialization() override {
        auto logger = g_active_mapping_node->get_logger();
        if (global_frame.empty()) {
            RCLCPP_WARN(logger, "global_frame is empty");
            return false;
        }
        if (robot_frame.empty()) {
            RCLCPP_WARN(logger, "robot_frame is empty");
            return false;
        }
        if (stop_exploration_ratio <= 0.0 || stop_exploration_ratio > 1.0) {
            RCLCPP_WARN(
                logger,
                "stop_exploration_ratio must be in (0, 1], got %f",
                stop_exploration_ratio);
            return false;
        }
        if (max_observed_area <= 0.0) {
            if (max_observed_area == 0.0) { max_observed_area = -1.0; }
            RCLCPP_WARN(
                logger,
                "max_observed_area (%f) is not positive, auto_stop will be disabled.",
                max_observed_area);
        }
        if (path_topic.path.empty()) {
            RCLCPP_WARN(logger, "path_topic.path is empty");
            return false;
        }
        if (goal_topic.path.empty()) {
            RCLCPP_WARN(logger, "goal_topic.path is empty");
            return false;
        }
        if (dist_topic.path.empty()) {
            RCLCPP_WARN(logger, "dist_topic.path is empty");
            return false;
        }
        if (observed_area_topic.path.empty()) {
            RCLCPP_WARN(logger, "observed_area_topic.path is empty");
            return false;
        }
        if (observed_ratio_topic.path.empty()) {
            RCLCPP_WARN(logger, "observed_ratio_topic.path is empty");
            return false;
        }
        if (replan_topic.path.empty()) {
            RCLCPP_WARN(logger, "replan_topic.path is empty");
            return false;
        }
        if (plan_srv.path.empty()) {
            RCLCPP_WARN(logger, "plan_srv.path is empty");
            return false;
        }
        if (enable_srv.path.empty()) {
            RCLCPP_WARN(logger, "enable_srv.path is empty");
            return false;
        }
        return true;
    }
};

template<typename Agent, typename Dtype, int Dim>
class ActiveMappingNode : public rclcpp::Node {
public:
    using AgentState = typename Agent::State;
    using Path = typename Agent::Path;
    using Pose = typename Agent::Pose;
    using Observation = typename Agent::Observation_t;

protected:
    ActiveMappingNodeConfig m_config_;

    std::shared_ptr<rclcpp::ParameterEventHandler> m_param_event_handler_;
    rclcpp::ParameterEventCallbackHandle::SharedPtr m_param_event_cb_handle_;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_observed_area_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_observed_ratio_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_replan_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_plan_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enable_srv_;

    std::shared_ptr<Agent> m_agent_ = nullptr;
    Pose m_last_pose_ = Pose::Identity();
    Path m_path_{};

    uint64_t m_step_ = 0;
    Dtype m_dist_ = 0;
    Dtype m_observed_area_ = 0;
    Dtype m_ratio_ = 0;
    bool m_replan_ = false;
    bool m_enabled_ = true;

public:
    explicit ActiveMappingNode(const std::string &node_name)
        : Node(node_name),
          m_param_event_handler_(std::make_shared<rclcpp::ParameterEventHandler>(this)),
          m_tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          m_tf_listener_(std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_)) {

        g_active_mapping_node = this;

        if (!m_config_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load ActiveMappingNodeConfig parameters");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded ActiveMappingNodeConfig:\n%s",
            m_config_.AsYamlString().c_str());

        // Set parameter change callback
#define SET_PARAM_IF(param, param_name, param_type, member)  \
    if (param.get_name() == param_name) {                    \
        if (param.get_type() == param_type) {                \
            member = param.get_value<decltype(member)>();    \
            RCLCPP_INFO(                                     \
                this->get_logger(),                          \
                "Parameter %s is set to %s",                 \
                param_name,                                  \
                param.value_to_string().c_str());            \
        } else {                                             \
            RCLCPP_WARN(                                     \
                this->get_logger(),                          \
                "Parameter %s has wrong type, expected %s.", \
                param_name,                                  \
                #param_type);                                \
        }                                                    \
        continue;                                            \
    }                                                        \
    (void) 0

        m_param_event_cb_handle_ = m_param_event_handler_->add_parameter_event_callback(
            [this](const rcl_interfaces::msg::ParameterEvent &event) {
                auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
                for (const auto &param: params) {
                    SET_PARAM_IF(
                        param,
                        "auto_replan",
                        rclcpp::ParameterType::PARAMETER_BOOL,
                        m_config_.auto_replan);
                    SET_PARAM_IF(
                        param,
                        "max_observed_area",
                        rclcpp::ParameterType::PARAMETER_DOUBLE,
                        m_config_.max_observed_area);
                }
            });
#undef SET_PARAM_IF

        m_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            m_config_.path_topic.path,
            m_config_.path_topic.GetQoS());
        m_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            m_config_.goal_topic.path,
            m_config_.goal_topic.GetQoS());
        m_dist_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            m_config_.dist_topic.path,
            m_config_.dist_topic.GetQoS());
        m_observed_area_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            m_config_.observed_area_topic.path,
            m_config_.observed_area_topic.GetQoS());
        m_observed_ratio_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            m_config_.observed_ratio_topic.path,
            m_config_.observed_ratio_topic.GetQoS());
        m_replan_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            m_config_.replan_topic.path,
            m_config_.replan_topic.GetQoS());
#ifdef ROS_HUMBLE
        m_plan_srv_ = this->create_service<std_srvs::srv::Trigger>(
            m_config_.plan_srv.path,
            std::bind(
                &ActiveMappingNode::CallbackSrvPlan,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_config_.plan_srv.GetQoS().get_rmw_qos_profile());
        m_enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
            m_config_.enable_srv.path,
            std::bind(
                &ActiveMappingNode::CallbackSrvEnable,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_config_.enable_srv.GetQoS().get_rmw_qos_profile());
#else
        m_plan_srv_ = this->create_service<std_srvs::srv::Trigger>(
            m_config_.plan_srv.path,
            std::bind(
                &ActiveMappingNode::CallbackSrvPlan,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_config_.plan_srv.GetQoS());
        m_enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
            m_config_.enable_srv.path,
            std::bind(
                &ActiveMappingNode::CallbackSrvEnable,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            m_config_.enable_srv.GetQoS());
#endif
    }

    virtual ~ActiveMappingNode() = default;

    void
    Step(const Pose &cur_pose, const Observation &observation, bool step_agent) {
        if (m_step_++) { m_dist_ += (cur_pose.col(Dim) - m_last_pose_.col(Dim)).norm(); }
        if (step_agent) { m_agent_->Step(cur_pose, observation); }
        m_last_pose_ = cur_pose;

        m_observed_area_ = ComputeObservedArea();
        m_ratio_ = m_observed_area_ / m_config_.max_observed_area;
        m_replan_ = m_agent_->ShouldReplan(cur_pose);
        if (m_ratio_ >= m_config_.stop_exploration_ratio) {
            RCLCPP_INFO(
                this->get_logger(),
                "Exploration completed (observed ratio: %.3f >= %.3f), stopping.",
                m_ratio_,
                m_config_.stop_exploration_ratio);
            m_replan_ = false;
            m_config_.auto_replan = false;
        }

        PublishStats();

        if (m_enabled_ && m_config_.auto_replan && m_replan_) {
            m_path_ = m_agent_->Plan(cur_pose);
            RCLCPP_INFO(
                this->get_logger(),
                "Auto replan at step %lu, new path with %lu waypoints.",
                m_step_,
                m_path_.size());
            m_replan_ = false;
            PublishPath(this->now());
        }
    }

    [[nodiscard]] virtual Dtype
    ComputeObservedArea() const = 0;

    void
    CallbackSrvPlan(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void) request;

        geometry_msgs::msg::TransformStamped pose;
        if (!GetPoseFromTf(this->now(), pose)) {
            response->success = false;
            response->message = "Failed to get robot pose from TF.";
            return;
        }

        Pose cur_pose = Pose::Identity();
        auto t = cur_pose.col(Dim);
        t[0] = pose.transform.translation.x;
        t[1] = pose.transform.translation.y;
        if constexpr (Dim == 3) {
            t[2] = pose.transform.translation.z;
            cur_pose.block(0, 3, 3, 3) = Eigen::Quaterniond(
                                             pose.transform.rotation.w,
                                             pose.transform.rotation.x,
                                             pose.transform.rotation.y,
                                             pose.transform.rotation.z)
                                             .toRotationMatrix()
                                             .cast<Dtype>();
        } else if constexpr (Dim == 2) {
            const double angle = std::acos(pose.transform.rotation.w) * 2;
            cur_pose(0, 0) = std::cos(angle);
            cur_pose(1, 0) = std::sin(angle);
            cur_pose(0, 1) = -std::sin(angle);
            cur_pose(1, 1) = std::cos(angle);
        }

        m_path_ = m_agent_->Plan(cur_pose);
        RCLCPP_INFO(
            this->get_logger(),
            "Plan service called at step %lu, new path with %lu waypoints.",
            m_step_,
            m_path_.size());
        PublishPath(this->now());

        response->success = true;
        response->message = "Planned path with " + std::to_string(m_path_.size()) + " waypoints.";
    }

    void
    CallbackSrvEnable(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        m_enabled_ = request->data;
        response->success = true;
        response->message = m_enabled_ ? "Planning enabled." : "Planning disabled.";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void
    PublishStats() {
        std_msgs::msg::Float64 dist_msg;
        dist_msg.data = static_cast<double>(m_dist_);
        m_dist_pub_->publish(dist_msg);

        std_msgs::msg::Float64 observed_area_msg;
        observed_area_msg.data = static_cast<double>(m_observed_area_);
        m_observed_area_pub_->publish(observed_area_msg);

        std_msgs::msg::Float64 ratio_msg;
        ratio_msg.data = static_cast<double>(m_ratio_);
        m_observed_ratio_pub_->publish(ratio_msg);

        std_msgs::msg::Bool replan_msg;
        replan_msg.data = m_replan_;
        m_replan_pub_->publish(replan_msg);
    }

    void
    PublishPath(const rclcpp::Time &stamp) {
        if (m_path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty path, not publishing.");
            return;
        }
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = m_config_.global_frame;
        path_msg.poses.resize(m_path_.size());
        for (size_t i = 0; i < m_path_.size(); ++i) {
            const Pose &pose = m_path_[i];
            geometry_msgs::msg::PoseStamped &pose_msg = path_msg.poses[i];
            pose_msg.header = path_msg.header;
            auto t = pose.col(Dim);
            pose_msg.pose.position.x = t[0];
            pose_msg.pose.position.y = t[1];
            if constexpr (Dim == 3) { pose_msg.pose.position.z = t[2]; }
            Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
            if constexpr (Dim == 2) {
                const double angle = std::atan2(pose(1, 0), pose(0, 0));
                q = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
            } else if constexpr (Dim == 3) {
                q = Eigen::Quaterniond(pose.block(0, 0, 3, 3));
            }
            q.normalize();
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
        }
        m_path_pub_->publish(path_msg);
        m_goal_pub_->publish(path_msg.poses.back());
    }

    bool
    GetPoseFromTf(const rclcpp::Time &time, geometry_msgs::msg::TransformStamped &pose) const {
        try {
            pose = m_tf_buffer_->lookupTransform(
                m_config_.global_frame,
                m_config_.robot_frame,
                time,
                rclcpp::Duration::from_seconds(5.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return false;
        }
        return true;
    }
};
