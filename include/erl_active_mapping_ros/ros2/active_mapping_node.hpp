#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <memory>

template<typename Agent, typename Dtype, int Dim>
class ActiveMappingNode : public rclcpp::Node {
public:
    using AgentState = typename Agent::State;
    using Path = typename Agent::Path;
    using Pose = typename Agent::Pose;
    using Observation = typename Agent::Observation_t;

protected:
    std::string m_global_frame_ = "map";
    std::string m_robot_frame_ = "base_link";
    bool m_auto_replan_ = false;
    double m_stop_exploration_ratio_ = 0.95;

    std::string m_default_qos_reliability_ = "reliable";
    std::string m_default_qos_durability_ = "volatile";

    std::string m_path_topic_ = "path";
    std::string m_path_topic_reliability_ = m_default_qos_reliability_;
    std::string m_path_topic_durability_ = m_default_qos_durability_;

    std::string m_dist_topic_ = "distance";
    std::string m_dist_topic_reliability_ = m_default_qos_reliability_;
    std::string m_dist_topic_durability_ = m_default_qos_durability_;

    std::string m_observed_area_topic_ = "observed_area";
    std::string m_observed_area_topic_reliability_ = m_default_qos_reliability_;
    std::string m_observed_area_topic_durability_ = m_default_qos_durability_;

    std::string m_observed_ratio_topic_ = "observed_ratio";
    std::string m_observed_ratio_topic_reliability_ = m_default_qos_reliability_;
    std::string m_observed_ratio_topic_durability_ = m_default_qos_durability_;

    std::string m_replan_topic_ = "replan";
    std::string m_replan_topic_reliability_ = m_default_qos_reliability_;
    std::string m_replan_topic_durability_ = m_default_qos_durability_;

    std::string m_plan_srv_name_ = "plan_path";

    double m_max_observed_area_ = 10.0;

    std::shared_ptr<rclcpp::ParameterEventHandler> m_param_event_handler_;
    rclcpp::ParameterEventCallbackHandle::SharedPtr m_param_event_cb_handle_;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_observed_area_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_observed_ratio_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_replan_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_plan_srv_;

    std::shared_ptr<Agent> m_agent_ = nullptr;
    Pose m_last_pose_ = Pose::Identity();
    Path m_path_{};

    uint64_t m_step_ = 0;
    Dtype m_dist_ = 0;
    Dtype m_observed_area_ = 0;
    Dtype m_ratio_ = 0;
    bool m_replan_ = false;

public:
    explicit ActiveMappingNode(const std::string &node_name)
        : Node(node_name),
          m_param_event_handler_(std::make_shared<rclcpp::ParameterEventHandler>(this)),
          m_tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          m_tf_listener_(std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_)) {

        this->declare_parameter("default_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("default_qos_durability", m_default_qos_durability_);
        this->get_parameter("default_qos_reliability", m_default_qos_reliability_);
        this->get_parameter("default_qos_durability", m_default_qos_durability_);

        this->declare_parameter("global_frame", m_global_frame_);
        this->declare_parameter("robot_frame", m_robot_frame_);
        this->declare_parameter("auto_replan", m_auto_replan_);
        this->declare_parameter("stop_exploration_ratio", m_stop_exploration_ratio_);

        this->declare_parameter("path_topic", m_path_topic_);
        this->declare_parameter("path_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("path_topic_durability", m_default_qos_durability_);

        this->declare_parameter("dist_topic", m_dist_topic_);
        this->declare_parameter("dist_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("dist_topic_durability", m_default_qos_durability_);

        this->declare_parameter("observed_area_topic", m_observed_area_topic_);
        this->declare_parameter("observed_area_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("observed_area_topic_durability", m_default_qos_durability_);

        this->declare_parameter("observed_ratio_topic", m_observed_ratio_topic_);
        this->declare_parameter("observed_ratio_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("observed_ratio_topic_durability", m_default_qos_durability_);

        this->declare_parameter("replan_topic", m_replan_topic_);
        this->declare_parameter("replan_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("replan_topic_durability", m_default_qos_durability_);

        this->declare_parameter("max_observed_area", m_max_observed_area_);

        // Get parameters
#define GET_PARAM(param_name, member)                           \
    if (!this->get_parameter(param_name, member)) {             \
        RCLCPP_WARN(                                            \
            this->get_logger(),                                 \
            "Failed to get parameter %s, using default value.", \
            param_name);                                        \
    }                                                           \
    (void) 0

        GET_PARAM("global_frame", m_global_frame_);
        GET_PARAM("robot_frame", m_robot_frame_);
        GET_PARAM("auto_replan", m_auto_replan_);
        GET_PARAM("stop_exploration_ratio", m_stop_exploration_ratio_);

        GET_PARAM("path_topic", m_path_topic_);
        GET_PARAM("path_topic_reliability", m_path_topic_reliability_);
        GET_PARAM("path_topic_durability", m_path_topic_durability_);

        GET_PARAM("dist_topic", m_dist_topic_);
        GET_PARAM("dist_topic_reliability", m_dist_topic_reliability_);
        GET_PARAM("dist_topic_durability", m_dist_topic_durability_);

        GET_PARAM("observed_area_topic", m_observed_area_topic_);
        GET_PARAM("observed_area_topic_reliability", m_observed_area_topic_reliability_);
        GET_PARAM("observed_area_topic_durability", m_observed_area_topic_durability_);

        GET_PARAM("observed_ratio_topic", m_observed_ratio_topic_);
        GET_PARAM("observed_ratio_topic_reliability", m_observed_ratio_topic_reliability_);
        GET_PARAM("observed_ratio_topic_durability", m_observed_ratio_topic_durability_);

        GET_PARAM("replan_topic", m_replan_topic_);
        GET_PARAM("replan_topic_reliability", m_replan_topic_reliability_);
        GET_PARAM("replan_topic_durability", m_replan_topic_durability_);

        GET_PARAM("max_observed_area", m_max_observed_area_);
#undef GET_PARAM

        RCLCPP_INFO(
            this->get_logger(),
            "Parameters:\n"
            "global_frame: %s\n"
            "robot_frame: %s\n"
            "auto_replan: %s\n"
            "stop_exploration_ratio: %f\n"
            "default_qos_reliability: %s\n"
            "default_qos_durability: %s\n"
            "path_topic: %s\n"
            "path_topic_reliability: %s\n"
            "path_topic_durability: %s\n"
            "dist_topic: %s\n"
            "dist_topic_reliability: %s\n"
            "dist_topic_durability: %s\n"
            "observed_area_topic: %s\n"
            "observed_area_topic_reliability: %s\n"
            "observed_area_topic_durability: %s\n"
            "observed_ratio_topic: %s\n"
            "observed_ratio_topic_reliability: %s\n"
            "observed_ratio_topic_durability: %s\n"
            "replan_topic: %s\n"
            "replan_topic_reliability: %s\n"
            "replan_topic_durability: %s\n"
            "max_observed_area: %f",
            m_global_frame_.c_str(),
            m_robot_frame_.c_str(),
            m_auto_replan_ ? "true" : "false",
            m_stop_exploration_ratio_,
            m_default_qos_reliability_.c_str(),
            m_default_qos_durability_.c_str(),
            m_path_topic_.c_str(),
            m_path_topic_reliability_.c_str(),
            m_path_topic_durability_.c_str(),
            m_dist_topic_.c_str(),
            m_dist_topic_reliability_.c_str(),
            m_dist_topic_durability_.c_str(),
            m_observed_area_topic_.c_str(),
            m_observed_area_topic_reliability_.c_str(),
            m_observed_area_topic_durability_.c_str(),
            m_observed_ratio_topic_.c_str(),
            m_observed_ratio_topic_reliability_.c_str(),
            m_observed_ratio_topic_durability_.c_str(),
            m_replan_topic_.c_str(),
            m_replan_topic_reliability_.c_str(),
            m_replan_topic_durability_.c_str(),
            m_max_observed_area_);

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
                        m_auto_replan_);
                    SET_PARAM_IF(
                        param,
                        "max_observed_area",
                        rclcpp::ParameterType::PARAMETER_DOUBLE,
                        m_max_observed_area_);
                }
            });
#undef SET_PARAM_IF

        m_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            m_path_topic_,
            GetQoS(m_path_topic_reliability_, m_path_topic_durability_));
        m_dist_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            m_dist_topic_,
            GetQoS(m_dist_topic_reliability_, m_dist_topic_durability_));
        m_observed_area_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            m_observed_area_topic_,
            GetQoS(m_observed_area_topic_reliability_, m_observed_area_topic_durability_));
        m_observed_ratio_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            m_observed_ratio_topic_,
            GetQoS(m_observed_ratio_topic_reliability_, m_observed_ratio_topic_durability_));
        m_replan_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            m_replan_topic_,
            GetQoS(m_replan_topic_reliability_, m_replan_topic_durability_));
    }

    virtual ~ActiveMappingNode() = default;

    void
    Step(const Pose &cur_pose, const Observation &observation, bool step_agent) {
        if (m_step_++) { m_dist_ += (cur_pose.col(Dim) - m_last_pose_.col(Dim)).norm(); }
        if (step_agent) { m_agent_->Step(cur_pose, observation); }
        m_last_pose_ = cur_pose;

        m_observed_area_ = ComputeObservedArea();
        m_ratio_ = m_observed_area_ / m_max_observed_area_;
        m_replan_ = m_agent_->ShouldReplan(cur_pose);
        if (m_ratio_ >= m_stop_exploration_ratio_) {
            RCLCPP_INFO(
                this->get_logger(),
                "Exploration completed (observed ratio: %.3f >= %.3f), stopping.",
                m_ratio_,
                m_stop_exploration_ratio_);
            m_replan_ = false;
            m_auto_replan_ = false;
        }

        PublishStats();

        if (m_auto_replan_ && m_replan_) {
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

    [[nodiscard]] Dtype
    ComputeObservedArea() const {
        auto log_odd_map = m_agent_->GetLogOddMap();
        std::size_t num_free_cells = log_odd_map->GetNumFreeCells();
        Eigen::Vector2<Dtype> res = log_odd_map->GetGridMapInfo()->Resolution();
        Dtype area = static_cast<Dtype>(num_free_cells) * res[0] * res[1];
        return area;
    }

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
        path_msg.header.frame_id = m_global_frame_;
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
    }

    bool
    GetPoseFromTf(const rclcpp::Time &time, geometry_msgs::msg::TransformStamped &pose) const {
        // get the latest transform from the tf buffer
        try {
            pose = m_tf_buffer_->lookupTransform(
                m_global_frame_,
                m_robot_frame_,
                time,
                rclcpp::Duration::from_seconds(5.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return false;
        }
        return true;
    }

    rclcpp::QoS
    GetQoS(const std::string &reliability, const std::string &durability) {
        rclcpp::QoS qos(1);
        if (reliability == "reliable") {
            qos.reliable();
        } else if (reliability == "best_effort") {
            qos.best_effort();
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown reliability %s, using reliable.",
                reliability.c_str());
            qos.reliable();
        }
        if (durability == "volatile") {
            qos.durability_volatile();
        } else if (durability == "transient_local") {
            qos.transient_local();
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown durability %s, using transient_local.",
                durability.c_str());
            qos.transient_local();
        }
        return qos;
    }
};
