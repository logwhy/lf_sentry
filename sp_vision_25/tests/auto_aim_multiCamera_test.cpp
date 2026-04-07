#include <fmt/core.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <thread>
#include <list>

#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char *argv[]) {
    tools::Exiter exiter;
    
    std::string config1 = "configs/cam1.yaml";
    std::string config2 = "configs/cam2.yaml";
    std::string shared_config = config1;
    
    std::unique_ptr<io::SNCamera> cam1, cam2;
    std::unique_ptr<auto_aim::Solver> solver1, solver2;
    std::unique_ptr<auto_aim::Tracker> tracker1, tracker2;
    std::unique_ptr<auto_aim::multithread::MultiThreadDetector> detector1, detector2;
    std::unique_ptr<auto_aim::Aimer> aimer;
    
    try {
        // --- 初始化相机 1 ---
        tools::logger()->info("正在初始化相机 1...");
        cam1 = std::make_unique<io::SNCamera>(config1);
        solver1 = std::make_unique<auto_aim::Solver>(config1);
        tracker1 = std::make_unique<auto_aim::Tracker>(config1, *solver1);
        detector1 = std::make_unique<auto_aim::multithread::MultiThreadDetector>(config1);
        
        // --- 关键：等待总线稳定再初始化第二个相机 ---
        tools::logger()->info("等待 USB 总线稳定 (2s)...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // --- 初始化相机 2 ---
        tools::logger()->info("正在初始化相机 2...");
        cam2 = std::make_unique<io::SNCamera>(config2);
        solver2 = std::make_unique<auto_aim::Solver>(config2);
        tracker2 = std::make_unique<auto_aim::Tracker>(config2, *solver2);
        detector2 = std::make_unique<auto_aim::multithread::MultiThreadDetector>(config2);
        
        // --- 初始化共享模块 ---
        aimer = std::make_unique<auto_aim::Aimer>(shared_config);
        tools::logger()->info("所有模块初始化成功！");
        
    } catch (const std::exception& e) {
        tools::logger()->error("初始化异常: {}", e.what());
        return -1;
    }
    
    // --- 采集线程 1 ---
    std::thread camera1_thread([&]() {
        cv::Mat frame;
        std::chrono::steady_clock::time_point timestamp;
        while (!exiter.exit()) {
            cam1->read(frame, timestamp);
            if (frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            detector1->push(frame, timestamp);
        }
    });
    
    // --- 采集线程 2 ---
    std::thread camera2_thread([&]() {
        cv::Mat frame;
        std::chrono::steady_clock::time_point timestamp;
        while (!exiter.exit()) {
            cam2->read(frame, timestamp);
            if (frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            detector2->push(frame, timestamp);
        }
    });
    
    // --- 主线程显示逻辑 ---
    cv::Mat f1, f2;
    int display_mode = 0;
    
    // 帧率计算相关变量
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    double fps = 0.0;
    
    // 用于记录最后一条命令（用于射击判断）
    io::Command last_command1, last_command2;
    
    while (!exiter.exit()) {
        auto result1 = detector1->debug_pop();
        auto result2 = detector2->debug_pop();
        
        // 处理相机 1 推理结果
        if (!std::get<0>(result1).empty()) {
            f1 = std::get<0>(result1).clone();
            auto& armors1 = std::get<1>(result1);
            auto& ts1 = std::get<2>(result1);
            solver1->set_R_gimbal2world({1, 0, 0, 0});
            
            // 更新跟踪器
            auto targets1 = tracker1->track(armors1, ts1);
            
            // 获取瞄准命令（假设使用固定距离27米，无IMU）
            auto command1 = aimer->aim(targets1, ts1, 27, false);
            
            // 简单射击逻辑（参考单相机版本）
            if (!targets1.empty() && aimer->debug_aim_point.valid &&
                std::abs(command1.yaw - last_command1.yaw) * 57.3 < 2) {
                command1.shoot = true;
            }
            if (command1.control) last_command1 = command1;
            
            // 在图像上显示命令信息
            std::string command_text1 = fmt::format("CAM1 - Yaw {:.2f}° | Pitch {:.2f}° | Shoot: {}",
                command1.yaw * 57.3, command1.pitch * 57.3,
                command1.shoot ? "YES" : "NO");
            cv::putText(f1, command_text1, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            
            // 绘制检测结果
            for (const auto& a : armors1) {
                cv::circle(f1, cv::Point2f(a.center_norm.x * f1.cols, a.center_norm.y * f1.rows), 
                          8, cv::Scalar(0, 0, 255), -1);
            }
        }
        
        // 处理相机 2 推理结果
        if (!std::get<0>(result2).empty()) {
            f2 = std::get<0>(result2).clone();
            auto& armors2 = std::get<1>(result2);
            auto& ts2 = std::get<2>(result2);
            solver2->set_R_gimbal2world({1, 0, 0, 0});
            
            // 更新跟踪器
            auto targets2 = tracker2->track(armors2, ts2);
            
            // 获取瞄准命令
            auto command2 = aimer->aim(targets2, ts2, 27, false);
            
            // 简单射击逻辑
            if (!targets2.empty() && aimer->debug_aim_point.valid &&
                std::abs(command2.yaw - last_command2.yaw) * 57.3 < 2) {
                command2.shoot = true;
            }
            if (command2.control) last_command2 = command2;
            
            // 在图像上显示命令信息
            std::string command_text2 = fmt::format("CAM2 - Yaw {:.2f}° | Pitch {:.2f}° | Shoot: {}",
                command2.yaw * 57.3, command2.pitch * 57.3,
                command2.shoot ? "YES" : "NO");
            cv::putText(f2, command_text2, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            
            // 绘制检测结果
            for (const auto& a : armors2) {
                cv::circle(f2, cv::Point2f(a.center_norm.x * f2.cols, a.center_norm.y * f2.rows), 
                          8, cv::Scalar(0, 255, 0), -1);
            }
        }
        
        // 计算帧率
        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - last_time).count();
        
        if (elapsed >= 1.0) {  // 每秒更新一次帧率
            fps = frame_count / elapsed;
            frame_count = 0;
            last_time = current_time;
        }
        
        // 窗口显示控制
        if (!f1.empty() && !f2.empty()) {
            cv::Mat display_mat;
            if (display_mode == 0) {
                cv::hconcat(f1, f2, display_mat);
            } else {
                display_mat = (display_mode == 1) ? f1 : f2;
            }
            
            // 显示帧率（在右下角）
            std::string fps_text = fmt::format("FPS: {:.1f}", fps);
            cv::putText(display_mat, fps_text, 
                       cv::Point(display_mat.cols - 150, display_mat.rows - 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            
            cv::Mat resized;
            cv::resize(display_mat, resized, cv::Size(), 0.5, 0.5);
            cv::imshow("Dual Cam Async Test", resized);
        }
        
        char key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        if (key == '1') display_mode = 1;
        if (key == '2') display_mode = 2;
        if (key == 'b') display_mode = 0;
    }
    
    camera1_thread.join();
    camera2_thread.join();
    return 0;
}