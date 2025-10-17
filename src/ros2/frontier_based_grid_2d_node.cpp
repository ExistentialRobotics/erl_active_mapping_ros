#include "erl_active_mapping/frontier_based_grid_2d.hpp"
#include "erl_active_mapping_ros/ros2/active_mapping_node.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

template<typename Dtype>
using Agent = erl::active_mapping::frontier_based::AgentFrontierBasedGrid2D<Dtype>;

template<typename Dtype>
class FrontierBasedGrid2dNode : public ActiveMappingNode<Agent<Dtype>, Dtype, 2> {
public:
    using Super = ActiveMappingNode<Agent<Dtype>, Dtype, 2>;
    using Agent_t = Agent<Dtype>;
    using AgentSetting = typename Agent_t::Setting;
    using GridMapInfo = typename Agent_t::GridMapInfo;
    using Pose = typename Agent_t::Pose;
    using Observation = typename Agent_t::Observation_t;

protected:
    std::string m_agent_config_file_;
    std::vector<double> m_map_min_ = {-10.1, -10.1};
    std::vector<double> m_map_max_ = {10.1, 10.1};
    double m_map_resolution_ = 0.05;

    bool m_use_external_map_ = false;
    std::string m_map_topic_ = "map";
    std::string m_map_topic_reliability_ = "reliable";
    std::string m_map_topic_durability_ = "transient_local";
    std::string m_scan_topic_ = "scan";
    std::string m_scan_topic_reliability_ = "reliable";
    std::string m_scan_topic_durability_ = "volatile";
    std::string m_internal_map_topic_ = "internal_map";
    std::string m_internal_map_topic_reliability_ = "reliable";
    std::string m_internal_map_topic_durability_ = "transient_local";

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_sub_ = nullptr;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_internal_map_pub_ = nullptr;

    std::shared_ptr<AgentSetting> m_agent_setting_ = std::make_shared<AgentSetting>();
    std::shared_ptr<GridMapInfo> m_grid_map_info_ = nullptr;

public:
    explicit FrontierBasedGrid2dNode(const std::string &node_name)
        : ActiveMappingNode<Agent<Dtype>, Dtype, 2>(node_name) {

        this->declare_parameter("agent_config_file", m_agent_config_file_);
        this->declare_parameter("map_min", m_map_min_);
        this->declare_parameter("map_max", m_map_max_);
        this->declare_parameter("map_resolution", m_map_resolution_);
        this->declare_parameter("use_external_map", m_use_external_map_);
        this->declare_parameter("map_topic", m_map_topic_);
        this->declare_parameter("map_topic_reliability", this->m_default_qos_reliability_);
        this->declare_parameter("map_topic_durability", this->m_default_qos_durability_);
        this->declare_parameter("scan_topic", m_scan_topic_);
        this->declare_parameter("scan_topic_reliability", this->m_default_qos_reliability_);
        this->declare_parameter("scan_topic_durability", this->m_default_qos_durability_);
        this->declare_parameter("internal_map_topic", m_internal_map_topic_);
        this->declare_parameter("internal_map_topic_reliability", this->m_default_qos_reliability_);
        this->declare_parameter("internal_map_topic_durability", this->m_default_qos_durability_);

        // Get parameters
#define GET_PARAM(param_name, member)                           \
    if (!this->get_parameter(param_name, member)) {             \
        RCLCPP_WARN(                                            \
            this->get_logger(),                                 \
            "Failed to get parameter %s, using default value.", \
            param_name);                                        \
    }                                                           \
    (void) 0
        GET_PARAM("agent_config_file", m_agent_config_file_);
        GET_PARAM("map_min", m_map_min_);
        GET_PARAM("map_max", m_map_max_);
        GET_PARAM("map_resolution", m_map_resolution_);
        GET_PARAM("use_external_map", m_use_external_map_);
        GET_PARAM("map_topic", m_map_topic_);
        GET_PARAM("map_topic_reliability", m_map_topic_reliability_);
        GET_PARAM("map_topic_durability", m_map_topic_durability_);
        GET_PARAM("scan_topic", m_scan_topic_);
        GET_PARAM("scan_topic_reliability", m_scan_topic_reliability_);
        GET_PARAM("scan_topic_durability", m_scan_topic_durability_);
        GET_PARAM("internal_map_topic", m_internal_map_topic_);
        GET_PARAM("internal_map_topic_reliability", m_internal_map_topic_reliability_);
        GET_PARAM("internal_map_topic_durability", m_internal_map_topic_durability_);
#undef GET_PARAM

        if (m_agent_config_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Agent configuration file is not set.");
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        if (!std::filesystem::exists(m_agent_config_file_)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Agent configuration file %s does not exist.",
                m_agent_config_file_.c_str());
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        if (m_map_min_.size() != 2 || m_map_max_.size() != 2) {
            RCLCPP_ERROR(
                this->get_logger(),
                "map_min and map_max should be of size 2, got %lu and %lu",
                m_map_min_.size(),
                m_map_max_.size());
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        if (m_map_resolution_ <= 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "map_resolution should be positive, got %f",
                m_map_resolution_);
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Parameters:\n"
            "agent_config_file: %s\n"
            "map_min: [%f, %f]\n"
            "map_max: [%f, %f]\n"
            "map_resolution: %f\n"
            "use_external_map: %s\n"
            "map_topic: %s\n"
            "map_topic_reliability: %s\n"
            "map_topic_durability: %s\n"
            "scan_topic: %s\n"
            "scan_topic_reliability: %s\n"
            "scan_topic_durability: %s\n"
            "internal_map_topic: %s\n"
            "internal_map_topic_reliability: %s\n"
            "internal_map_topic_durability: %s\n",
            m_agent_config_file_.c_str(),
            m_map_min_[0],
            m_map_min_[1],
            m_map_max_[0],
            m_map_max_[1],
            m_map_resolution_,
            (m_use_external_map_ ? "true" : "false"),
            m_map_topic_.c_str(),
            m_map_topic_reliability_.c_str(),
            m_map_topic_durability_.c_str(),
            m_scan_topic_.c_str(),
            m_scan_topic_reliability_.c_str(),
            m_scan_topic_durability_.c_str(),
            m_internal_map_topic_.c_str(),
            m_internal_map_topic_reliability_.c_str(),
            m_internal_map_topic_durability_.c_str());

        // Load agent configuration
        try {
            if (!m_agent_setting_->FromYamlFile(m_agent_config_file_)) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Failed to load agent configuration from %s",
                    m_agent_config_file_.c_str());
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded agent configuration from %s:\n%s",
                m_agent_config_file_.c_str(),
                m_agent_setting_->AsYamlString().c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to load agent configuration file %s: %s",
                m_agent_config_file_.c_str(),
                e.what());
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        m_grid_map_info_ = std::make_shared<GridMapInfo>(
            Eigen::Vector2<Dtype>(m_map_min_[0], m_map_min_[1]),
            Eigen::Vector2<Dtype>(m_map_max_[0], m_map_max_[1]),
            Eigen::Vector2<Dtype>(m_map_resolution_, m_map_resolution_),
            Eigen::Vector2i::Zero());
        this->m_agent_ = std::make_shared<Agent_t>(m_agent_setting_, m_grid_map_info_);

        if (m_use_external_map_) {
            // Subscribe to map topic
            m_map_sub_ = this->template create_subscription<nav_msgs::msg::OccupancyGrid>(
                m_map_topic_,
                Super::GetQoS(m_map_topic_reliability_, m_map_topic_durability_),
                std::bind(&FrontierBasedGrid2dNode::CallbackMap, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to map topic %s", m_map_topic_.c_str());
        } else {
            // Subscribe to laser scan topic
            m_laser_sub_ = this->template create_subscription<sensor_msgs::msg::LaserScan>(
                m_scan_topic_,
                Super::GetQoS(m_scan_topic_reliability_, m_scan_topic_durability_),
                std::bind(&FrontierBasedGrid2dNode::CallbackLaser, this, std::placeholders::_1));
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribed to laser scan topic %s",
                m_scan_topic_.c_str());
        }

        m_internal_map_pub_ = this->template create_publisher<nav_msgs::msg::OccupancyGrid>(
            m_internal_map_topic_,
            Super::GetQoS(m_internal_map_topic_reliability_, m_internal_map_topic_durability_));
    }

    void
    CallbackMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (msg == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Received null map message.");
            return;
        }
        const Dtype res = static_cast<Dtype>(msg->info.resolution);
        const Dtype min_x = static_cast<Dtype>(msg->info.origin.position.x);
        const Dtype min_y = static_cast<Dtype>(msg->info.origin.position.y);

        auto grid_map_info = std::make_shared<GridMapInfo>(
            Eigen::Vector2<Dtype>(min_x, min_y),
            Eigen::Vector2<Dtype>(
                min_x + static_cast<Dtype>(msg->info.width) * res,
                min_y + static_cast<Dtype>(msg->info.height) * res),
            Eigen::Vector2<Dtype>(res, res),
            Eigen::Vector2i(msg->info.width, msg->info.height));

        geometry_msgs::msg::TransformStamped pose;
        this->GetPoseFromTf(msg->header.stamp, pose);
        const Eigen::Vector2<Dtype> agent_pos(
            static_cast<Dtype>(pose.transform.translation.x),
            static_cast<Dtype>(pose.transform.translation.y));
        Eigen::Matrix3<Dtype> rotation = Eigen::Quaternion<Dtype>(
                                             pose.transform.rotation.w,
                                             pose.transform.rotation.x,
                                             pose.transform.rotation.y,
                                             pose.transform.rotation.z)
                                             .toRotationMatrix();
        const Dtype theta = std::atan2(rotation(1, 0), rotation(0, 0));
        Eigen::Map<const Eigen::MatrixX<int8_t>> occ_map(
            msg->data.data(),
            msg->info.width,    // x
            msg->info.height);  // y
        this->m_agent_->GetLogOddMap()
            ->template LoadExternalPossibilityMap<int8_t>(agent_pos, theta, occ_map, grid_map_info);
        PublishInternalMap(msg->header.stamp);

        Pose cur_pose;
        const Dtype cos_theta = std::cos(theta);
        const Dtype sin_theta = std::sin(theta);
        cur_pose << cos_theta, -sin_theta, agent_pos[0], sin_theta, cos_theta, agent_pos[1];
        // we load the external map. no observation is needed. no need to step the agent.
        this->Step(cur_pose, {} /* observation */, false /* step_agent */);
    }

    void
    CallbackLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Received null laser scan message.");
            return;
        }

        geometry_msgs::msg::TransformStamped pose;
        if (!this->GetPoseFromTf(msg->header.stamp, pose)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to get robot pose at time %u.%u from TF.",
                msg->header.stamp.sec,
                msg->header.stamp.nanosec);
            return;
        }
        const Eigen::Vector2<Dtype> agent_pos(
            static_cast<Dtype>(pose.transform.translation.x),
            static_cast<Dtype>(pose.transform.translation.y));
        Eigen::Matrix2<Dtype> rotation = Eigen::Quaternion<Dtype>(
                                             pose.transform.rotation.w,
                                             pose.transform.rotation.x,
                                             pose.transform.rotation.y,
                                             pose.transform.rotation.z)
                                             .toRotationMatrix()
                                             .template block<2, 2>(0, 0);

        Observation points(2, msg->ranges.size());
        float angle = msg->angle_min;
        long idx = 0;
        for (const float &range: msg->ranges) {
            if (!std::isfinite(range)) {
                angle += msg->angle_increment;
                continue;
            }
            auto p = points.col(idx);
            p[0] = static_cast<Dtype>(range * std::cos(angle));
            p[1] = static_cast<Dtype>(range * std::sin(angle));
            p = rotation * p + agent_pos;
            angle += msg->angle_increment;
            ++idx;
        }

        points.conservativeResize(Eigen::NoChange, idx);
        Pose cur_pose;
        cur_pose.template block<2, 2>(0, 0) = rotation;
        cur_pose.col(2) = agent_pos;
        this->Step(cur_pose, points, true /* step_agent */);  // step the agent to update the map
        PublishInternalMap(msg->header.stamp);
    }

    void
    PublishInternalMap(const rclcpp::Time &stamp) {
        auto log_odd_map = this->m_agent_->GetLogOddMap();
        cv::Mat occ_map = log_odd_map->GetOccupancyMap();
        if (occ_map.empty()) {
            RCLCPP_WARN(this->get_logger(), "Internal map is empty, cannot publish.");
            return;
        }
        auto grid_map_info = log_odd_map->GetGridMapInfo();

        // occ_map: row is x, col is y, row-major
        // nav_msgs/OccupancyGrid: row is y, col is x, row-major
        // occ_map_eigen: row is x, col is y, col-major
        // the required storage order is the same between occ_map_eigen and nav_msgs/OccupancyGrid
        Eigen::MatrixX8U occ_map_eigen;
        cv::cv2eigen(occ_map, occ_map_eigen);

        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = this->m_global_frame_;
        msg.info.resolution = static_cast<float>(grid_map_info->Resolution(0));
        msg.info.width = static_cast<uint32_t>(occ_map.cols);
        msg.info.height = static_cast<uint32_t>(occ_map.rows);
        msg.info.origin.position.x = static_cast<float>(grid_map_info->Min(0));
        msg.info.origin.position.y = static_cast<float>(grid_map_info->Min(1));
        msg.info.origin.position.z = 0.0f;
        msg.info.origin.orientation.w = 1.0f;
        msg.info.origin.orientation.x = 0.0f;
        msg.info.origin.orientation.y = 0.0f;
        msg.info.origin.orientation.z = 0.0f;
        msg.data.resize(msg.info.width * msg.info.height);
        std::memcpy(msg.data.data(), occ_map_eigen.data(), sizeof(int8_t) * msg.data.size());

        m_internal_map_pub_->publish(msg);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create a node to read parameters
    auto temp_node = rclcpp::Node::make_shared("astar_2d_node");

    bool double_precision = false;
    temp_node->declare_parameter<bool>("double_precision", false);
    double_precision = temp_node->get_parameter("double_precision").as_bool();
    RCLCPP_INFO(
        temp_node->get_logger(),
        "Using %s precision.",
        double_precision ? "double" : "single");
    temp_node.reset();  // release the temporary node

    std::shared_ptr<rclcpp::Node> node;
    if (double_precision) {
        node = std::make_shared<FrontierBasedGrid2dNode<double>>("frontier_based_grid_2d_node");
    } else {
        node = std::make_shared<FrontierBasedGrid2dNode<float>>("frontier_based_grid_2d_node");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
