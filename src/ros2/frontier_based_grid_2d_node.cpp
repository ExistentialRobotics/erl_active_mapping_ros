#include "erl_active_mapping/frontier_based_grid_2d.hpp"
#include "erl_active_mapping_ros/ros2/active_mapping_node.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

template<typename Dtype>
using Agent = erl::active_mapping::frontier_based::AgentFrontierBasedGrid2D<Dtype>;

using namespace erl::common;
using namespace erl::common::ros_params;

struct FrontierBasedGrid2dNodeConfig : public Yamlable<FrontierBasedGrid2dNodeConfig> {
    std::string agent_config_file;
    std::vector<double> map_min = {-10.1, -10.1};
    std::vector<double> map_max = {10.1, 10.1};
    double map_resolution = 0.05;
    bool use_external_map = false;
    Ros2TopicParams map_topic{"map"};
    Ros2TopicParams scan_topic{"scan"};
    Ros2TopicParams internal_map_topic{"internal_map"};

    ERL_REFLECT_SCHEMA(
        FrontierBasedGrid2dNodeConfig,
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, agent_config_file),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_min),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_max),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_resolution),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, use_external_map),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_topic),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, scan_topic),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, internal_map_topic));

    bool
    PostDeserialization() override {
        auto logger = g_active_mapping_node->get_logger();

        if (agent_config_file.empty()) {
            RCLCPP_WARN(logger, "agent_config_file is not set");
            return false;
        }
        if (!std::filesystem::exists(agent_config_file)) {
            RCLCPP_WARN(
                logger,
                "Agent configuration file %s does not exist",
                agent_config_file.c_str());
            return false;
        }
        if (map_min.size() != 2 || map_max.size() != 2) {
            RCLCPP_WARN(
                logger,
                "map_min and map_max should be of size 2, got %lu and %lu",
                map_min.size(),
                map_max.size());
            return false;
        }
        if (map_resolution <= 0) {
            RCLCPP_WARN(
                logger,
                "map_resolution should be positive, got %f",
                map_resolution);
            return false;
        }
        if (map_topic.path.empty()) {
            RCLCPP_WARN(logger, "map_topic.path is empty");
            return false;
        }
        if (scan_topic.path.empty()) {
            RCLCPP_WARN(logger, "scan_topic.path is empty");
            return false;
        }
        if (internal_map_topic.path.empty()) {
            RCLCPP_WARN(logger, "internal_map_topic.path is empty");
            return false;
        }
        return true;
    }
};

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
    FrontierBasedGrid2dNodeConfig m_derived_config_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_sub_ = nullptr;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_internal_map_pub_ = nullptr;

    std::shared_ptr<AgentSetting> m_agent_setting_ = std::make_shared<AgentSetting>();
    std::shared_ptr<GridMapInfo> m_grid_map_info_ = nullptr;

public:
    explicit FrontierBasedGrid2dNode(const std::string &node_name)
        : ActiveMappingNode<Agent<Dtype>, Dtype, 2>(node_name) {

        if (!m_derived_config_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load FrontierBasedGrid2dNodeConfig parameters");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded FrontierBasedGrid2dNodeConfig:\n%s",
            m_derived_config_.AsYamlString().c_str());

        // Load agent configuration
        try {
            if (!m_agent_setting_->FromYamlFile(m_derived_config_.agent_config_file)) {
                RCLCPP_FATAL(
                    this->get_logger(),
                    "Failed to load agent configuration from %s",
                    m_derived_config_.agent_config_file.c_str());
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded agent configuration from %s:\n%s",
                m_derived_config_.agent_config_file.c_str(),
                m_agent_setting_->AsYamlString().c_str());
        } catch (const std::exception &e) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load agent configuration file %s: %s",
                m_derived_config_.agent_config_file.c_str(),
                e.what());
            rclcpp::shutdown();
            return;
        }

        m_grid_map_info_ = std::make_shared<GridMapInfo>(
            Eigen::Vector2<Dtype>(m_derived_config_.map_min[0], m_derived_config_.map_min[1]),
            Eigen::Vector2<Dtype>(m_derived_config_.map_max[0], m_derived_config_.map_max[1]),
            Eigen::Vector2<Dtype>(m_derived_config_.map_resolution, m_derived_config_.map_resolution),
            Eigen::Vector2i::Zero());
        this->m_agent_ = std::make_shared<Agent_t>(m_agent_setting_, m_grid_map_info_);

        if (m_derived_config_.use_external_map) {
            m_map_sub_ = this->template create_subscription<nav_msgs::msg::OccupancyGrid>(
                m_derived_config_.map_topic.path,
                m_derived_config_.map_topic.GetQoS(),
                std::bind(&FrontierBasedGrid2dNode::CallbackMap, this, std::placeholders::_1));
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribed to map topic %s",
                m_derived_config_.map_topic.path.c_str());
        } else {
            m_laser_sub_ = this->template create_subscription<sensor_msgs::msg::LaserScan>(
                m_derived_config_.scan_topic.path,
                m_derived_config_.scan_topic.GetQoS(),
                std::bind(&FrontierBasedGrid2dNode::CallbackLaser, this, std::placeholders::_1));
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribed to laser scan topic %s",
                m_derived_config_.scan_topic.path.c_str());
        }

        m_internal_map_pub_ = this->template create_publisher<nav_msgs::msg::OccupancyGrid>(
            m_derived_config_.internal_map_topic.path,
            m_derived_config_.internal_map_topic.GetQoS());
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
        msg.header.frame_id = this->m_config_.global_frame;
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
