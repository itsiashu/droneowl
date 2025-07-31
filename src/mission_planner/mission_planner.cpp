#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <GeographicLib/LocalCartesian.hpp>

class MissionLoader : public rclcpp::Node {
public:
    MissionLoader() : Node("mission_loader") {
        this->declare_parameter<std::string>("mission_file", "config/mission.yaml");
        std::string file = this->get_parameter("mission_file").as_string();
        RCLCPP_INFO(get_logger(), "Loading mission from %s", file.c_str());

        pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        YAML::Node config = YAML::LoadFile(file);
        auto waypoints = config["geo_waypoints"];
        if (!waypoints) {
            RCLCPP_ERROR(get_logger(), "No geo_waypoints in YAML!");
            return;
        }

        // Use first waypoint as origin for ENU
        double lat0 = waypoints[0]["lat"].as<double>();
        double lon0 = waypoints[0]["lon"].as<double>();
        double alt0 = waypoints[0]["alt"].as<double>();
        GeographicLib::LocalCartesian proj(lat0, lon0, alt0);

        // Timer publishes waypoints one by one
        index_ = 0;
        waypoints_.clear();
        for (auto wp : waypoints) {
            double lat = wp["lat"].as<double>();
            double lon = wp["lon"].as<double>();
            double alt = wp["alt"].as<double>();
            double x, y, z;
            proj.Forward(lat, lon, alt, x, y, z);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.w = 1.0;
            waypoints_.push_back(pose);
        }

        timer_ = create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&MissionLoader::publish_next_goal, this));
    }

private:
    void publish_next_goal() {
        if (index_ >= waypoints_.size()) {
            RCLCPP_INFO(get_logger(), "All goals published.");
            return;
        }
        auto goal = waypoints_[index_++];
        goal.header.stamp = now();
        RCLCPP_INFO(get_logger(), "Publishing goal %d at (%.1f, %.1f, %.1f)",
                    index_, goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        pub_->publish(goal);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t index_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionLoader>());
    rclcpp::shutdown();
    return 0;
}