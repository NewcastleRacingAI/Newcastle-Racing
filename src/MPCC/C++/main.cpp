// Copyright 2019 Alexander Liniger
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "Tests/spline_test.h" // 导入样条测试
#include "Tests/model_integrator_test.h" // 导入模型积分器测试
#include "Tests/constratins_test.h" // 导入约束条件测试
#include "Tests/cost_test.h" // 导入代价测试

#include "MPC/mpc.h" // 导入 MPC 控制器的主要逻辑
#include "Model/integrator.h" // 导入状态积分器
#include "Params/track.h" // 导入赛道数据
#include "Plotting/plotting.h" // 导入绘图组件

#include <nlohmann/json.hpp> // 导入 JSON 库用于解析配置文件
using json = nlohmann::json; // 简化 JSON 类型命名

int main1() {
    using namespace mpcc; // 使用 mpcc 命名空间

    // 打开并读取 JSON 配置文件
    std::ifstream iConfig("Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig; // 解析 JSON 文件

    // 从 JSON 配置中提取各个文件路径
    PathToJson json_paths {
        jsonConfig["model_path"],
        jsonConfig["cost_path"],
        jsonConfig["bounds_path"],
        jsonConfig["track_path"],
        jsonConfig["normalization_path"]
    };

    // 各组件测试（已注释以简化主程序）
    // std::cout << testSpline() << std::endl;
    // std::cout << testArcLengthSpline(json_paths) << std::endl;
    // std::cout << testIntegrator(json_paths) << std::endl;
    // std::cout << testLinModel(json_paths) << std::endl;
    // std::cout << testAlphaConstraint(json_paths) << std::endl;
    // std::cout << testTireForceConstraint(json_paths) << std::endl;
    // std::cout << testTrackConstraint(json_paths) << std::endl;
    // std::cout << testCost(json_paths) << std::endl;

    // 初始化积分器，用于状态更新
    Integrator integrator = Integrator(jsonConfig["Ts"], json_paths);

    // 初始化绘图器，用于结果绘制
    Plotting plotter = Plotting(jsonConfig["Ts"], json_paths);

    // 初始化赛道数据
    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack(); // 获取赛道 X 和 Y 坐标

    // 初始化控制器的日志记录
    std::list<MPCReturn> log;

    // 初始化 MPC 控制器
    MPC mpc(jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], jsonConfig["Ts"], json_paths);
    mpc.setTrack(track_xy.X, track_xy.Y); // 设置赛道轨迹

    // 计算初始方向和初始状态
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0), track_xy.X(1) - track_xy.X(0));
    State x0 = {
        track_xy.X(0), // 初始 X 坐标
        track_xy.Y(0), // 初始 Y 坐标
        phi_0, // 初始方向
        jsonConfig["v0"], // 初始速度
        0, // 初始横向速度
        0, // 初始横摆速度
        0, // 初始转角
        0.5, // 初始油门位置
        0, // 初始制动
        jsonConfig["v0"] // 目标速度
    };

    // 循环模拟 MPC 控制器的运行
    for (int i = 0; i < jsonConfig["n_sim"]; i++) {
        // 获取 MPC 计算的控制输出
        MPCReturn mpc_sol = mpc.runMPC(x0);
        
        // 使用积分器更新车辆状态
        x0 = integrator.simTimeStep(x0, mpc_sol.u0, jsonConfig["Ts"]);
        
        // 将结果添加到日志
        log.push_back(mpc_sol);
    }

    // 使用绘图器绘制控制结果
    plotter.plotRun(log, track_xy);
    plotter.plotSim(log, track_xy);

    // 计算并输出平均和最大 MPC 计算时间
    double mean_time = 0.0;
    double max_time = 0.0;
    for (MPCReturn log_i : log) {
        mean_time += log_i.time_total;
        if (log_i.time_total > max_time) {
            max_time = log_i.time_total;
        }
    }

    // 输出计算时间信息
    std::cout << "mean nmpc time " << mean_time / double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;

    return 0;
}
