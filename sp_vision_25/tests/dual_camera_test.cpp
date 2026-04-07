#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fmt/core.h>
#include <cstring>
#include <vector>

#include "tools/crc.hpp"   // 使用项目自带的 CRC 校验工具
#include "tools/logger.hpp"

// --- 协议结构体定义 (严格对应 Self_aim.h) ---
#pragma pack(push, 1)
typedef struct {
    uint8_t     mode;
    float       q[4];
    float       yaw;
    float       yaw_vel;
    float       pitch;
    float       pitch_vel;
    float       bullet_speed;
    uint16_t    bullet_count;
} OutputData;

typedef struct {
    uint8_t       mode;
    float         yaw;
    float         yaw_vel;
    float         yaw_acc;
    float         pitch;
    float         pitch_vel;
    float         pitch_acc;
} InputData;

typedef struct { uint8_t s; uint8_t p; } FrameHeader;
typedef struct { uint16_t crc16; } FrameTailer;

typedef struct {            
    FrameHeader frame_header;
    InputData   input_data;
    FrameTailer frame_tailer;   
} RECEIVE_DATA; // 模拟器接收（上位机发出的指令）

typedef struct {         
    FrameHeader frame_header;
    OutputData  output_data;
    FrameTailer frame_tailer;
} SEND_DATA;    // 模拟器发送（给上位机的反馈）
#pragma pack(pop)

// 读满 len 个字节（阻塞），成功返回 true
static bool read_full(int fd, uint8_t* buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = ::read(fd, buf + got, len - got);
        if (n <= 0) return false;   // 对于阻塞fd，<=0 通常表示出错或被关闭
        got += static_cast<size_t>(n);
    }
    return true;
}

// 从字节流中同步到帧头 'S''P'，然后读取整个 RECEIVE_DATA 帧到 out_buf
static bool read_receive_frame(int fd, uint8_t* out_buf, size_t out_len) {
    if (out_len != sizeof(RECEIVE_DATA)) return false;

    // 1) 先同步帧头：逐字节找 'S''P'
    uint8_t b = 0;
    uint8_t prev = 0;
    while (true) {
        ssize_t n = ::read(fd, &b, 1);
        if (n <= 0) return false;

        if (prev == 'S' && b == 'P') {
            out_buf[0] = 'S';
            out_buf[1] = 'P';
            break;
        }
        prev = b;
    }

    // 2) 再读剩下的字节（结构体总长 - 2）
    return read_full(fd, out_buf + 2, out_len - 2);
}


int main() {
    // 串口路径：请根据 socat 生成的路径修改（模拟器通常连 pts 的一端）
    const char* serial_port = "/dev/pts/5"; 
    int fd = open(serial_port, O_RDWR | O_NONBLOCK);
    
    if (fd < 0) {
        std::cerr << fmt::format("无法打开虚拟串口 {}, 请确认 socat 是否启动！", serial_port) << std::endl;
        return -1;
    }

    std::cout << ">>> 虚拟下位机模拟器已启动 [SP_VISION SIMULATOR]" << std::endl;
    std::cout << ">>> 监听端口: " << serial_port << std::endl;

    SEND_DATA status_pkg;
    RECEIVE_DATA cmd_pkg;
    uint8_t rx_buf[sizeof(RECEIVE_DATA)];

    while (true) {
        // --- 1. 模拟下位机向上位机发送数据 (反馈 IMU 姿态) ---
        SEND_DATA status_pkg;
        std::memset(&status_pkg, 0, sizeof(status_pkg));  // ★关键：先清零
        status_pkg.frame_header.s = 'S';
        status_pkg.frame_header.p = 'P';
        status_pkg.output_data.mode = 1;
        status_pkg.output_data.q[0] = 1.0f;
        status_pkg.output_data.bullet_speed = 27.0f;
        // status_pkg.frame_header.s = 'S'; // FRAME_HEADER_S
        // status_pkg.frame_header.p = 'P'; // FRAME_HEADER_P
        
        // status_pkg.output_data.mode = 1;
        // status_pkg.output_data.q[0] = 1.0f; // 模拟静止四元数 [w=1, x=0, y=0, z=0]
        // status_pkg.output_data.q[1] = 0.0f;
        // status_pkg.output_data.q[2] = 0.0f;
        // status_pkg.output_data.q[3] = 0.0f;
        // status_pkg.output_data.bullet_speed = 27.0f; // 模拟弹速

        // 计算发送包的 CRC16
        uint32_t send_len = sizeof(SEND_DATA) - 2; 
        status_pkg.frame_tailer.crc16 = tools::get_crc16((uint8_t*)&status_pkg, send_len);

        write(fd, &status_pkg, sizeof(SEND_DATA));

        // --- 2. 模拟下位机接收上位机的控制指令 ---
        if (read_receive_frame(fd, rx_buf, sizeof(RECEIVE_DATA))) {

            // 再做 CRC 校验（你原来的 check_crc16 逻辑可以保留）
            if (tools::check_crc16(rx_buf, sizeof(RECEIVE_DATA))) {
                std::memcpy(&cmd_pkg, rx_buf, sizeof(RECEIVE_DATA));

                // 帧头再确认一下（保险）
                if (cmd_pkg.frame_header.s == 'S' && cmd_pkg.frame_header.p == 'P') {
                    std::cout << fmt::format(
                        "[CMD] yaw(deg)={:.2f}, pitch(deg)={:.2f}, mode={}",
                        cmd_pkg.input_data.yaw*57.3f,
                        cmd_pkg.input_data.pitch*57.3f,
                        (cmd_pkg.input_data.mode == 1 ? "ON" : "OFF")
                    ) << std::endl;
                } else {
                    std::cout << "[Warning] 收到一帧但帧头不匹配（可能上位机协议不一致）" << std::endl;
                }

            } else {
                std::cout << "[Warning] 收到一帧，但 CRC 校验失败（可能仍有错位/协议不一致）" << std::endl;
            }
        }
        // else: 没读到完整帧（阻塞 fd 下通常不会频繁发生，除非对端关闭/异常）


        // 同步下位机频率：30ms
        usleep(3000); 
    }

    close(fd);
    return 0;
}
