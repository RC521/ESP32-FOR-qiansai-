#ifndef SJJC_H
#define SJJC_H

#include "mcp_server.h"
#include <esp_log.h>
#include "driver/uart.h"
#include <string>
#include <vector>
#include <string.h>

// ================= TAG =================
#ifdef TAG
#undef TAG
#endif
#define TAG "SJJC"

// ================= UART 配置 =================
#define TXD_PIN (GPIO_NUM_38)
#define RXD_PIN (GPIO_NUM_39)
#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024)
// ============================================


// ===== 位置解析（只负责位置）=====
static char parse_location(const std::string& s) {
    if (s.find("1") != std::string::npos || s.find("一") != std::string::npos || s.find("左") != std::string::npos)
        return '1';
    if (s.find("2") != std::string::npos || s.find("二") != std::string::npos || s.find("中") != std::string::npos)
        return '2';
    if (s.find("3") != std::string::npos || s.find("三") != std::string::npos || s.find("右") != std::string::npos)
        return '3';
    return 'X';
}

class Uart_Detector {
private:
    uart_port_t uart_num_;

    // ===== 核心执行函数（所有颜色共用）=====
    std::string detect(char color_code, const PropertyList& args) {

        std::string location;

        try {
            location = args["location"].value<std::string>();
        } catch (...) {
            try {
                location = std::to_string(args["location"].value<int>());
            } catch (...) {
                location = "";
            }
        }

        char loc_code = parse_location(location);

        ESP_LOGW(
            TAG,
            "解析结果：color=%c , location='%s' → %c",
            color_code, location.c_str(), loc_code
        );

        if (loc_code == 'X') {
            return "{\"结果\":\"失败\",\"播报\":\"位置不明确\"}";
        }

        // ===== 构造串口指令（颜色 + 位置）=====
        char cmd[8];
        snprintf(cmd, sizeof(cmd), "%c%c\n", color_code, loc_code);

        ESP_LOGW(TAG, "检测指令：%s", cmd);

        uint8_t data[64];
        memset(data, 0, sizeof(data)); // 初始化接收缓冲区

        // ===== 最多尝试 6 次 =====
        for (int attempt = 1; attempt <= 6; attempt++) {

            ESP_LOGW(TAG, "第 %d / 6 次检测", attempt);

            // ✅ 每一轮都清空 RX，防止历史数据污染
            uart_flush_input(uart_num_);

            // 发送指令
            uart_write_bytes(uart_num_, cmd, strlen(cmd));

            int elapsed = 0;
            const int timeout_ms = 500;

            while (elapsed < timeout_ms) {

                int len = uart_read_bytes(
                    uart_num_,
                    data,
                    sizeof(data) - 1,
                    pdMS_TO_TICKS(100)
                );

                if (len > 0) {
                    data[len] = '\0'; // 确保字符串以'\0'结尾
                    std::string recv_data((char*)data);
                    ESP_LOGW(TAG, "收到返回数据：%s", recv_data.c_str());

                    // 1. 检测是否返回 no/NO（不区分大小写）
                    std::string recv_lower = recv_data;
                    std::transform(recv_lower.begin(), recv_lower.end(), recv_lower.begin(), ::tolower);
                    if (recv_lower.find("no") != std::string::npos) {
                        ESP_LOGW(TAG, "检测到返回no，未识别到目标");
                        return "{\"结果\":\"失败\",\"播报\":\"未识别到目标\"}";
                    }

                    // 2. 原逻辑：前两个字符匹配则判定成功
                    if (len >= 2 && data[0] == cmd[0] && data[1] == cmd[1]) {
                        ESP_LOGW(TAG, "匹配成功");
                        return "{\"结果\":\"成功\",\"播报\":\"已稳定识别到目标\"}";
                    }
                }

                elapsed += 100;
            }
        }

        ESP_LOGW(TAG, "6 次检测均失败");
        return "{\"结果\":\"失败\",\"播报\":\"检测超时，未识别到目标\"}";
    }


    // ===== 新增：机械臂移动核心函数 =====
    std::string detect_move(char color_code, const PropertyList& args) {

        std::string location;

        try {
            location = args["location"].value<std::string>();
        } catch (...) {
            try {
                location = std::to_string(args["location"].value<int>());
            } catch (...) {
                location = "";
            }
        }

        char loc_code = parse_location(location);

        ESP_LOGW(
            TAG,
            "移动解析结果：color=%c , location='%s' → %c",
            color_code, location.c_str(), loc_code
        );

        if (loc_code == 'X') {
            return "{\"结果\":\"失败\",\"播报\":\"位置不明确，无法移动\"}";
        }

        // ===== 构造机械臂移动指令（颜色 + 位置 + M）=====
        char cmd[8];
        snprintf(cmd, sizeof(cmd), "%c%cM\n", color_code, loc_code); // 指令格式如 R1M、G2M、B3M

        ESP_LOGW(TAG, "机械臂移动指令：%s", cmd);

        uint8_t data[64];
        memset(data, 0, sizeof(data));

        // ===== 最多尝试 3 次移动指令 =====
        for (int attempt = 1; attempt <= 3; attempt++) {

            ESP_LOGW(TAG, "第 %d / 3 次发送移动指令", attempt);

            uart_flush_input(uart_num_); // 清空接收缓冲区
            uart_write_bytes(uart_num_, cmd, strlen(cmd)); // 发送移动指令

            int elapsed = 0;
            const int timeout_ms = 1000; // 移动指令超时时间更长（1秒）

            while (elapsed < timeout_ms) {

                int len = uart_read_bytes(
                    uart_num_,
                    data,
                    sizeof(data) - 1,
                    pdMS_TO_TICKS(100)
                );

                if (len > 0) {
                    data[len] = '\0';
                    std::string recv_data((char*)data);
                    ESP_LOGW(TAG, "移动指令返回数据：%s", recv_data.c_str());

                    // 判定移动成功（可根据OpenMV/机械臂实际返回值调整，比如"ok"/"success"）
                    std::string recv_lower = recv_data;
                    std::transform(recv_lower.begin(), recv_lower.end(), recv_lower.begin(), ::tolower);
                    if (recv_lower.find("ok") != std::string::npos || recv_lower.find("success") != std::string::npos) {
                        ESP_LOGW(TAG, "机械臂移动成功");
                        return "{\"结果\":\"成功\",\"播报\":\"已将指定位置的目标移动至指定位置\"}";
                    }

                    // 检测到no，说明移动目标未识别，移动失败
                    if (recv_lower.find("no") != std::string::npos) {
                        ESP_LOGW(TAG, "移动目标未识别，移动失败");
                        return "{\"结果\":\"失败\",\"播报\":\"未识别到移动目标，移动失败\"}";
                    }
                }

                elapsed += 100;
            }
        }

        ESP_LOGW(TAG, "移动指令发送失败");
        return "{\"结果\":\"失败\",\"播报\":\"移动指令超时，机械臂未响应\"}";
    }



public:
    explicit Uart_Detector(uart_port_t uart_num = UART_NUM)
        : uart_num_(uart_num) {

        // ===== UART 初始化 =====
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

        ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(
            uart_num_, TXD_PIN, RXD_PIN,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE
        ));
        ESP_ERROR_CHECK(uart_driver_install(
            uart_num_, BUF_SIZE * 2,
            0, 0, NULL, 0
        ));

        auto& server = McpServer::GetInstance();

        // ===== 只需要 location 参数 =====
        std::vector<Property> props;
        props.push_back(Property("location", (PropertyType)1)); // string

        // ===== 红色 =====
        server.AddTool(
            "视觉检测.红色",
            "检测指定位置是否存在红色目标",
            PropertyList(props),
            [this](const PropertyList& args) {
                return detect('R', args);
            }
        );

        // ===== 蓝色 =====
        server.AddTool(
            "视觉检测.蓝色",
            "检测指定位置是否存在蓝色目标",
            PropertyList(props),
            [this](const PropertyList& args) {
                return detect('B', args);
            }
        );

        // ===== 绿色 =====
        server.AddTool(
            "视觉检测.绿色",
            "检测指定位置是否存在绿色目标",
            PropertyList(props),
            [this](const PropertyList& args) {
                return detect('G', args);
            }
        );

        // ===== 新增：机械臂移动功能 =====
        // 移动红色方块
        server.AddTool(
            "视觉检测.移动红色方块",
            "将指定位置的红色方块移动至指定位置",
            PropertyList(props),
            [this](const PropertyList& args) {
                return detect_move('R', args);
            }
        );

        // 移动蓝色方块
        server.AddTool(
            "视觉检测.移动蓝色方块",
            "将指定位置的蓝色方块移动至指定位置",
            PropertyList(props),
            [this](const PropertyList& args) {
                return detect_move('B', args);
            }
        );

        // 移动绿色方块
        server.AddTool(
            "视觉检测.移动绿色方块",
            "将指定位置的绿色方块移动至指定位置",
            PropertyList(props),
            [this](const PropertyList& args) {
                return detect_move('G', args);
            }
        );

    }
};

#endif // SJJC_H