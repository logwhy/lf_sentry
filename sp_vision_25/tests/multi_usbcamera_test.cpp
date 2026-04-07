#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <yaml-cpp/yaml.h> // 必须安装并链接 yaml-cpp 库
#include <fstream>  // 修复：解决 std::ifstream 报错
#include <unistd.h>   // 修复：提供 getcwd 函数支持

#ifdef __cplusplus
extern "C" {
#endif
#include "MvCameraControl.h"
#ifdef __cplusplus
}
#endif

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

// --- 配置结构体定义 ---
struct CamConfig {
    std::string serial;
    float exposure;
    float gain;
};

struct AppSettings {
    bool display;
    std::string save_path;
};

class HikCamera {
private:
    void* handle_;
    std::string serial_;
    int width_;
    int height_;
    
public:
    HikCamera() : handle_(nullptr), width_(0), height_(0) {}
    
    ~HikCamera() {
        close();
    }
    
    // 修改后的 open 函数：接受 CamConfig 结构体作为参数
    bool open(const CamConfig& config) {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        
        // 1. 枚举设备
        int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
            tools::logger()->error("未找到USB设备");
            return false;
        }
        
        int targetIndex = -1;
        this->serial_ = config.serial;

        // 2. 根据配置文件中的序列号匹配设备
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
                std::string sn = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
                if (sn == serial_) {
                    targetIndex = i;
                    break;
                }
            }
        }
        
        if (targetIndex == -1) {
            tools::logger()->error("相机序列号 [{}] 未连接", serial_);
            return false;
        }
        
        // 3. 打开设备
        nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[targetIndex]);
        if (MV_OK != nRet) return false;
        
        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet) {
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }
        
        // 4. 设置配置参数（解耦核心）
        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        MV_CC_SetFloatValue(handle_, "ExposureTime", config.exposure);
        MV_CC_SetFloatValue(handle_, "Gain", config.gain);
        
        // 5. 获取图像分辨率
        MVCC_INTVALUE stParam;
        MV_CC_GetIntValue(handle_, "Width", &stParam);
        width_ = stParam.nCurValue;
        MV_CC_GetIntValue(handle_, "Height", &stParam);
        height_ = stParam.nCurValue;
        
        // 6. 开始采集
        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet) {
            close();
            return false;
        }
        
        tools::logger()->info("相机 {} 初始化成功: 曝光={}us, 增益={}", serial_, config.exposure, config.gain);
        return true;
    }
    
    bool read(cv::Mat& image) {
        if (!handle_) return false;
        
        MV_FRAME_OUT stImageInfo = {0};
        int nRet = MV_CC_GetImageBuffer(handle_, &stImageInfo, 1000);
        if (MV_OK != nRet) return false;
        
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        int buffer_size = stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight * 3;
        std::vector<unsigned char> buffer(buffer_size);
        
        stConvertParam.nWidth = stImageInfo.stFrameInfo.nWidth;
        stConvertParam.nHeight = stImageInfo.stFrameInfo.nHeight;
        stConvertParam.pSrcData = stImageInfo.pBufAddr;
        stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;
        stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = buffer.data();
        stConvertParam.nDstBufferSize = buffer_size;
        
        bool success = false;
        if (MV_OK == MV_CC_ConvertPixelType(handle_, &stConvertParam)) {
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                          stImageInfo.stFrameInfo.nWidth,
                          CV_8UC3,
                          buffer.data()).clone();
            success = true;
        }
        
        MV_CC_FreeImageBuffer(handle_, &stImageInfo);
        return success;
    }
    
    void close() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
    }
    
    const std::string& getSerial() const { return serial_; }
};

int main(int argc, char* argv[]) {
    tools::Exiter exiter;
    
    // --- 1. 配置文件路径搜索逻辑 ---
    std::vector<std::string> search_paths = {
        "../configs/two_camera.yaml",
        "./configs/two_camera.yaml",
        "../../configs/two_camera.yaml",
        "configs/two_camera.yaml"
    };

    std::string final_path = "";
    for (const auto& path : search_paths) {
        std::ifstream f(path);
        if (f.good()) {
            final_path = path;
            break;
        }
    }

    if (final_path.empty()) {
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            tools::logger()->error("找不到配置文件！当前工作目录: {}", cwd);
        }
        return -1;
    }

    // --- 2. 解析 YAML 配置 ---
    CamConfig cfg1, cfg2;
    AppSettings settings;
    try {
        YAML::Node root = YAML::LoadFile(final_path);
        settings.display = root["settings"]["display_enabled"].as<bool>();
        settings.save_path = root["settings"]["save_path"].as<std::string>("./");
        cfg1 = {root["camera1"]["serial_number"].as<std::string>(), root["camera1"]["exposure_time"].as<float>(), root["camera1"]["gain"].as<float>()};
        cfg2 = {root["camera2"]["serial_number"].as<std::string>(), root["camera2"]["exposure_time"].as<float>(), root["camera2"]["gain"].as<float>()};
    } catch (const std::exception& e) {
        tools::logger()->error("解析 YAML 异常: {}", e.what());
        return -1;
    }

    // --- 3. 初始化相机 ---
    HikCamera cam1;
    HikCamera cam2;
    if (!cam1.open(cfg1) || !cam2.open(cfg2)) return -1;

    // --- 4. 循环逻辑 ---
    cv::Mat img1, img2;
    int frame_count = 0;
    bool paused = false;
    int display_mode = 0; // 0: 并排, 1: 仅相机1, 2: 仅相机2
    int last_mode = -1;   // 用于检测模式切换

    tools::logger()->info("开始采集。按 '1'/'2' 切换单显，按 'b' 并排显示，按 'q' 退出。");

    while (!exiter.exit()) {
        if (paused) {
            cv::waitKey(100);
            continue;
        }

        if (cam1.read(img1) && cam2.read(img2)) {
            frame_count++;

            if (settings.display) {
                // --- 模式切换清理逻辑 ---
                if (display_mode != last_mode) {
                    cv::destroyAllWindows(); // 切换模式时关闭所有旧窗口
                    last_mode = display_mode;
                }

                cv::Mat out_img;
                std::string win_name;

                // --- 根据模式准备图像 ---
                if (display_mode == 0) {
                    win_name = "Dual_View";
                    cv::hconcat(img1, img2, out_img);
                } else if (display_mode == 1) {
                    win_name = "Camera_1";
                    out_img = img1.clone();
                } else {
                    win_name = "Camera_2";
                    out_img = img2.clone();
                }

                // --- 图像缩放逻辑 (核心：解决窗口太大) ---
                // 将显示尺寸缩小为原来的 0.5 倍（可根据实际需要调整 0.4 或 0.6）
                cv::Mat resized_img;
                double scale = 0.5; 
                cv::resize(out_img, resized_img, cv::Size(), scale, scale);

                // 在缩放后的图上画点信息（可选）
                cv::putText(resized_img, "Mode: " + std::to_string(display_mode), cv::Point(20, 40), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

                cv::imshow(win_name, resized_img);

                // --- 按键处理 ---
                char key = cv::waitKey(1);
                if (key == 'q' || key == 27) break;
                if (key == 'p') paused = !paused;
                if (key == '1') display_mode = 1;
                if (key == '2') display_mode = 2;
                if (key == 'b') display_mode = 0;
                if (key == 's') {
                    cv::imwrite("cam1_cap.png", img1);
                    cv::imwrite("cam2_cap.png", img2);
                    tools::logger()->info("已保存原始分辨率图像");
                }
            }
        }
    }

    cam1.close();
    cam2.close();
    cv::destroyAllWindows();
    return 0;
}
