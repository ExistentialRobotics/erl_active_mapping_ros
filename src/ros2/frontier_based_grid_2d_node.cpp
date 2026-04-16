#include "erl_active_mapping/frontier_based_grid_2d.hpp"
#include "erl_active_mapping_ros/ros2/active_mapping_node.hpp"
#include "erl_geometry/bresenham_2d.hpp"
#include "erl_geometry_msgs/msg/frontier_array.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

template<typename Dtype>
using Agent = erl::active_mapping::frontier_based::AgentFrontierBasedGrid2D<Dtype>;

using namespace erl::common;
using namespace erl::common::ros_params;

struct FrontierBasedGrid2dNodeConfig
    : public Yamlable<FrontierBasedGrid2dNodeConfig, ActiveMappingNodeConfig> {
    std::string agent_config_file;
    std::vector<double> map_min = {-10.1, -10.1};
    std::vector<double> map_max = {10.1, 10.1};
    double map_resolution = 0.05;
    bool use_external_map = false;
    bool use_external_frontier = false;
    Ros2TopicParams map_topic{"map"};
    Ros2TopicParams scan_topic{"scan"};
    Ros2TopicParams frontier_topic{"frontier"};
    Ros2TopicParams internal_map_topic{"internal_map"};

    ERL_REFLECT_SCHEMA(
        FrontierBasedGrid2dNodeConfig,
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, agent_config_file),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_min),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_max),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_resolution),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, use_external_map),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, use_external_frontier),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, map_topic),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, scan_topic),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, frontier_topic),
        ERL_REFLECT_MEMBER(FrontierBasedGrid2dNodeConfig, internal_map_topic));

    bool
    PostDeserialization() override {
        if (!ActiveMappingNodeConfig::PostDeserialization()) {
            return false;
        }

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
            RCLCPP_WARN(logger, "map_resolution should be positive, got %f", map_resolution);
            return false;
        }
        if (use_external_map && map_topic.path.empty()) {
            RCLCPP_WARN(logger, "map_topic.path is empty");
            return false;
        }
        if (!use_external_map && scan_topic.path.empty()) {
            RCLCPP_WARN(logger, "scan_topic.path is empty");
            return false;
        }
        if (use_external_frontier && frontier_topic.path.empty()) {
            RCLCPP_WARN(logger, "frontier_topic.path is empty");
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
    // map subscription to the external map for updating the internal map. Used if use_external_map
    // is true.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub_ = nullptr;
    // laser scan subscription for updating the internal map. Used if use_external_map is false.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_sub_ = nullptr;
    // frontier subscription for loading external frontiers. Used if use_external_frontier is true.
    rclcpp::Subscription<erl_geometry_msgs::msg::FrontierArray>::SharedPtr m_frontier_sub_ =
        nullptr;
    // for visualization and debugging, we also publish the internal map as nav_msgs/OccupancyGrid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_internal_map_pub_ = nullptr;

    std::shared_ptr<AgentSetting> m_agent_setting_ = std::make_shared<AgentSetting>();
    std::shared_ptr<GridMapInfo> m_grid_map_info_ = nullptr;

    nav_msgs::msg::OccupancyGrid::SharedPtr m_latest_map_msg_ = nullptr;
    erl_geometry_msgs::msg::FrontierArray::SharedPtr m_latest_frontier_msg_ = nullptr;
    bool m_map_initialized_ = false;  // true after the first external map is loaded

public:
    explicit FrontierBasedGrid2dNode(const std::string &node_name)
        : ActiveMappingNode<Agent<Dtype>, Dtype, 2>(node_name) {

        if (!m_derived_config_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load FrontierBasedGrid2dNodeConfig parameters");
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
            Eigen::Vector2<Dtype>(
                m_derived_config_.map_resolution,
                m_derived_config_.map_resolution),
            Eigen::Vector2i::Zero());
        this->m_agent_ = std::make_shared<Agent_t>(m_agent_setting_, m_grid_map_info_);
        m_map_initialized_ = !m_derived_config_.use_external_map;

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

        if (m_derived_config_.use_external_frontier) {
            m_frontier_sub_ =
                this->template create_subscription<erl_geometry_msgs::msg::FrontierArray>(
                    m_derived_config_.frontier_topic.path,
                    m_derived_config_.frontier_topic.GetQoS(),
                    std::bind(
                        &FrontierBasedGrid2dNode::CallbackFrontier,
                        this,
                        std::placeholders::_1));
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribed to frontier topic %s",
                m_derived_config_.frontier_topic.path.c_str());
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
        m_latest_map_msg_ = msg;
        ProcessExternalSources();
    }

    void
    CallbackLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Received null laser scan message.");
            return;
        }

        Pose cur_pose;
        if (!GetAgentPose(msg->header.stamp, cur_pose)) { return; }
        Eigen::Matrix2<Dtype> rotation = cur_pose.template block<2, 2>(0, 0);
        Eigen::Vector2<Dtype> agent_pos = cur_pose.col(2);

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
        cur_pose.template block<2, 2>(0, 0) = rotation;
        cur_pose.col(2) = agent_pos;
        this->Step(cur_pose, points, true /* step_agent */);  // step the agent to update the map
        PublishInternalMap(msg->header.stamp);
    }

    void
    CallbackFrontier(const erl_geometry_msgs::msg::FrontierArray::SharedPtr msg) {
        if (msg == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Received null frontier message.");
            return;
        }
        if (msg->dim != 2) {
            RCLCPP_WARN(
                this->get_logger(),
                "Received frontier with dimension %u, but only 2D frontier is supported.",
                msg->dim);
            return;
        }
        m_latest_frontier_msg_ = msg;
        ProcessExternalSources();
    }

    void
    ProcessExternalSources() {
        bool updated = false;

        const rclcpp::Time stamp = this->now();
        Pose cur_pose;
        if (!GetAgentPose(stamp, cur_pose)) { return; }

        // 1. Load external map if a new one arrived
        if (m_latest_map_msg_) {
            const auto &msg = m_latest_map_msg_;
            const Dtype res = static_cast<Dtype>(msg->info.resolution);
            const Dtype min_x = static_cast<Dtype>(msg->info.origin.position.x);
            const Dtype min_y = static_cast<Dtype>(msg->info.origin.position.y);

            const auto grid_map_info = std::make_shared<GridMapInfo>(
                Eigen::Vector2i(msg->info.width, msg->info.height),
                Eigen::Vector2<Dtype>(min_x, min_y),
                Eigen::Vector2<Dtype>(
                    min_x + static_cast<Dtype>(msg->info.width) * res,
                    min_y + static_cast<Dtype>(msg->info.height) * res));

            const Eigen::Vector2<Dtype> agent_pos = cur_pose.col(2);
            const Dtype theta = std::atan2(cur_pose(1, 0), cur_pose(0, 0));

            Eigen::Map<const Eigen::MatrixX<int8_t>> occ_map(
                msg->data.data(),
                msg->info.width,
                msg->info.height);

            this->m_agent_->GetLogOddMap()->template LoadExternalPossibilityMap<int8_t>(
                agent_pos,
                theta,
                occ_map,
                grid_map_info);
            this->m_agent_->SetEnvOutdated(true);
            m_map_initialized_ = true;

            PublishInternalMap(stamp);
            m_latest_map_msg_ = nullptr;
            updated = true;
        }

        // 2. Load external frontiers if a new one arrived (requires map to be initialized)
        if (m_latest_frontier_msg_ && m_map_initialized_) {
            const auto &msg = m_latest_frontier_msg_;
            const auto grid_map_info = this->m_agent_->GetLogOddMap()->GetGridMapInfo();
            const int width = grid_map_info->Shape(0);
            const int height = grid_map_info->Shape(1);

            using Frontier = typename Agent_t::Frontier;
            std::vector<Frontier> frontiers;

            for (const auto &msg_frontier: msg->frontiers) {
                const auto num_vertices = static_cast<long>(msg_frontier.vertices.size());
                if (num_vertices == 0) { continue; }

                Frontier frontier;
                frontier.points.resize(2, num_vertices);
                long n = 0;
                Eigen::Vector2i start(
                    grid_map_info->MeterToGridAtDim(
                        static_cast<Dtype>(msg_frontier.vertices[0].x),
                        0),
                    grid_map_info->MeterToGridAtDim(
                        static_cast<Dtype>(msg_frontier.vertices[0].y),
                        1));
                for (long i = 1; i < num_vertices; ++i) {
                    Eigen::Vector2i end(
                        grid_map_info->MeterToGridAtDim(
                            static_cast<Dtype>(msg_frontier.vertices[i].x),
                            0),
                        grid_map_info->MeterToGridAtDim(
                            static_cast<Dtype>(msg_frontier.vertices[i].y),
                            1));

                    Eigen::Matrix2Xi seg_pts = erl::geometry::Bresenham2D(start, end);
                    if (n + seg_pts.cols() > frontier.points.cols()) {
                        long new_cols = std::max(frontier.points.cols() * 2, n + seg_pts.cols());
                        frontier.points.conservativeResize(Eigen::NoChange, new_cols);
                    }
                    for (long j = 0; j < seg_pts.cols(); ++j) {
                        auto p = seg_pts.col(j);
                        if (p[0] < 0 || p[0] >= width || p[1] < 0 || p[1] >= height) { continue; }
                        frontier.points.col(n++) = p;
                    }
                    start = end;
                }
                frontier.points.conservativeResize(Eigen::NoChange, n);
                frontier.score = msg_frontier.score;
                frontiers.push_back(std::move(frontier));
            }
            this->m_agent_->SetFrontiers(frontiers);
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded %zu frontiers from external source.",
                frontiers.size());
            m_latest_frontier_msg_ = nullptr;
            updated = true;
        }

        // 3. Step the agent if anything was updated
        if (!updated) { return; }
        this->Step(cur_pose, {} /* observation */, false /* step_agent */);
    }

    bool
    GetAgentPose(const rclcpp::Time &stamp, Pose &out_pose) {
        geometry_msgs::msg::TransformStamped pose;
        if (!this->GetPoseFromTf(stamp, pose)) { return false; }
        Eigen::Matrix3<Dtype> rotation = Eigen::Quaternion<Dtype>(
                                             pose.transform.rotation.w,
                                             pose.transform.rotation.x,
                                             pose.transform.rotation.y,
                                             pose.transform.rotation.z)
                                             .toRotationMatrix();
        const Dtype theta = std::atan2(rotation(1, 0), rotation(0, 0));

        const Dtype cos_theta = std::cos(theta);
        const Dtype sin_theta = std::sin(theta);
        // clang-format off
        out_pose << cos_theta, -sin_theta, static_cast<Dtype>(pose.transform.translation.x),
                    sin_theta, cos_theta, static_cast<Dtype>(pose.transform.translation.y);
        // clang-format on
        return true;
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

    [[nodiscard]] Dtype
    ComputeObservedArea() const override {
        auto log_odd_map = this->m_agent_->GetLogOddMap();
        std::size_t num_free_cells = log_odd_map->GetNumFreeCells();
        Eigen::Vector2<Dtype> res = log_odd_map->GetGridMapInfo()->Resolution();
        Dtype area = static_cast<Dtype>(num_free_cells) * res[0] * res[1];
        return area;
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
