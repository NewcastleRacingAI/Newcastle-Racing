#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <Eigen/Dense>
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/path_with_boundaries.hpp"

#include <vector>
#include "MPC/mpc.h" // 导入模型预测控制器的主要逻辑
#include "Model/integrator.h" // 导入状态积分器，用于车辆状态的时间更新
#include "Params/track.h" // 导入赛道参数
#include "Plotting/plotting.h" // 导入绘图组件
#include <nlohmann/json.hpp> // 导入 JSON 库用于解析配置文件
#include <fstream> // 导入文件流操作库

using json = nlohmann::json;

std::string getCurrentWorkingDirectory() {
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        return std::string(cwd);
    } else {
        perror("getcwd() error");
        return "";
    }
}

std::string getParentPath(const std::string& path) {
    std::size_t pos = path.find_last_of('/');
    if (pos != std::string::npos) {
        std::size_t pos2 = path.find_last_of('/', pos - 1);
        if (pos2 != std::string::npos) {
            return path.substr(0, pos2);
        }
    }
    return "";
}

class MPCCController : public rclcpp::Node {
public:
    MPCCController()
        : Node("mpcc_controller_node"),
        last_steering_angle(0.0),
        track(getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/track.json"), 
        total_distance(0.0), 
        D(0.5),
        path_direction(0.0, 0.0){
        last_position = Eigen::Vector2d(0.0, 0.0); 
        initializeSubscribersAndPublishers();
    }

private:
    void initializeSubscribersAndPublishers() {
        auto jsonConfig = loadConfig(getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/config.json");

        // 参数初始化
        mpc = std::make_unique<mpcc::MPC>(jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], jsonConfig["Ts"], loadJsonPaths());
        
        path_subscriber = this->create_subscription<eufs_msgs::msg::PathWithBoundaries>(
            "/path", 1, std::bind(&MPCCController::path_callback, this, std::placeholders::_1));
        car_state_subscriber = this->create_subscription<eufs_msgs::msg::CarState>(
            "/odometry_integration/car_state", 1, std::bind(&MPCCController::car_state_callback, this, std::placeholders::_1));
        control_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
    }

    json loadConfig(const std::string& filename) {
        std::ifstream iConfig(filename);
        if (!iConfig.is_open()) {
            RCLCPP_FATAL(this->get_logger(), "Unable to open configuration file: %s", filename.c_str());
            throw std::runtime_error("Unable to open configuration file.");
        }
        json j;
        try {
            iConfig >> j;
        } catch (json::parse_error& e) {
            RCLCPP_FATAL(this->get_logger(), "JSON parse error in configuration file: %s", e.what());
            throw;
        }
        return j;
    }

    mpcc::PathToJson loadJsonPaths() {
        return {
            getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/model.json",
            getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/cost.json",
            getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/bounds.json",
            getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/track.json",
            getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/normalization.json"
        };
    }

    // 路径数据回调函数
    void path_callback(const eufs_msgs::msg::PathWithBoundaries::SharedPtr msg) {


        std::vector<double> x_coords, y_coords;
        x_coords.clear();
        y_coords.clear();

        for (const auto &point : msg->path) {
            x_coords.push_back(point.x);
            y_coords.push_back(point.y);

        }


        
        std::vector<double> x_inner_coords, y_inner_coords;
        std::vector<double> x_outer_coords, y_outer_coords;

        x_inner_coords.clear();
        y_inner_coords.clear();
        x_outer_coords.clear();
        y_outer_coords.clear();

        for (const auto &point : msg->sorted_right) {
            x_inner_coords.push_back(point.x);
            y_inner_coords.push_back(point.y);
        }

        for (const auto &point : msg->sorted_left) {
            x_outer_coords.push_back(point.x);
            y_outer_coords.push_back(point.y);
        }

        Eigen::Map<Eigen::VectorXd> x_eigen(x_coords.data(), x_coords.size());
        Eigen::Map<Eigen::VectorXd> y_eigen(y_coords.data(), y_coords.size());
        if (x_coords.size() > 1) {  // 确保有足够的点来计算方向
            path_direction = Eigen::Vector2d(x_eigen(1) - x_eigen(0), y_eigen(1) - y_eigen(0));
        }
        Eigen::Map<Eigen::VectorXd> x_inner_eigen(x_inner_coords.data(), x_inner_coords.size());
        Eigen::Map<Eigen::VectorXd> y_inner_eigen(y_inner_coords.data(), x_inner_coords.size());

        Eigen::Map<Eigen::VectorXd> x_outer_eigen(x_outer_coords.data(), x_outer_coords.size());
        Eigen::Map<Eigen::VectorXd> y_outer_eigen(y_outer_coords.data(), x_outer_coords.size());


        
        track.setTrackData(x_eigen, y_eigen, x_inner_eigen, y_inner_eigen, x_outer_eigen, y_outer_eigen);
        track_received = true;
  
    }
  // 计算车辆在路径方向上的速度
    double calculatePathDirectionalSpeed(const Eigen::Vector2d& direction, double vx, double vy) {
        Eigen::Vector2d velocity(vx, vy);
        double speed_in_direction = velocity.dot(direction.normalized());
        return speed_in_direction;
    }

    // 小车状态数据回调函数
    void car_state_callback(const eufs_msgs::msg::CarState::SharedPtr msg) {
        // 提取车辆位置
        car_position = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
        // 计算从上一位置到当前位置的距离
        if (track_received) {  // 确保已经接收到至少一次位置更新
            double distance = (car_position - last_position).norm();
            total_distance += distance;
        }

        // 更新上一位置
        last_position = car_position;

        // 提取纵向速度
        vx = msg->twist.twist.linear.x;
        // 提取横向速度
        double vy = msg->twist.twist.linear.y;
        // 提取偏航率
        double r = msg->twist.twist.angular.z;

        // 从四元数中提取车辆航向角
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;
        double sin_yaw = 2.0 * (w * z + x * y);
        double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
        double phi = std::atan2(sin_yaw, cos_yaw);
    

        // 计算在路径方向上的速度
        double vs = calculatePathDirectionalSpeed(path_direction, vx, vy);

        // 调用更新控制逻辑函数
        update_control(vx, vy, r, phi,vs);
        // 可以在此输出或记录总行驶距离
        RCLCPP_INFO(this->get_logger(), "Total distance traveled: %f meters", total_distance);
    }


    // 更新控制逻辑
    void update_control(double vx, double vy, double r, double phi, double vs) {
            if (!track_received) {
                return;
            }

            mpcc::TrackPos track_xy = track.getTrack();
            mpc->setTrack(track_xy.X, track_xy.Y);

            mpcc::State current_state = {
                car_position.x(), car_position.y(), 
                phi, 
                vx, vy, 
                r, total_distance, D, last_steering_angle, vs
            };

            mpcc::MPCReturn mpc_result = mpc->runMPC(current_state);
            //构建控制消息
            auto control_msg = ackermann_msgs::msg::AckermannDriveStamped();
            control_msg.header.stamp = this->now();
            control_msg.drive.steering_angle = mpc_result.u0.dDelta; // 前轮转向角度
            control_msg.drive.acceleration =  mpc_result.u0.dVs;
            //control_msg.drive.duty_cycle = mpc_result.u0.dD;
            // 发布控制消息
            control_publisher->publish(control_msg);
            // 更新转向角度状态
            last_steering_angle = mpc_result.u0.dDelta;
            D = mpc_result.u0.dD;
            std::cout << "X: " << car_position.x() << "\n";
            std::cout << "Y: " << car_position.y() << "\n";
            std::cout << "phi: " << phi << "\n";
            std::cout << "vx: " << vx << "\n";
            std::cout << "vy: " << vy << "\n";
            std::cout << "r: " << r << "\n";
            std::cout << "vs: " << vs << "\n";
            std::cout << "D: " << mpc_result.u0.dD << "\n";
            std::cout << "mpc_result.u0.dVs: " << mpc_result.u0.dVs << "\n";
            std::cout << "last_steering_angle: " << last_steering_angle << "\n";
    }

    // 成员变量定义
    rclcpp::Subscription<eufs_msgs::msg::PathWithBoundaries>::SharedPtr path_subscriber;
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr car_state_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_publisher;

    std::unique_ptr<mpcc::MPC> mpc;
    mpcc::Track track;
    bool track_received;

    Eigen::Vector2d car_position;
    Eigen::Vector2d car_direction;
    double vx;
    double previous_speed;
    double previous_acceleration;
    double last_steering_angle;
     // 其他成员变量
    Eigen::Vector2d last_position;  // 上一次车辆位置
    Eigen::Vector2d path_direction; // 添加路径方向为成员变量
    double D; 
    double total_distance;          // 累计行驶的总距离
    json jsonConfig;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCCController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
