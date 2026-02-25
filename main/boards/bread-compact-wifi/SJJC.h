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

            // 只要前两个字符是 R1 就判定成功
            if (len >= 2 &&
                data[0] == cmd[0] &&
                data[1] == cmd[1]) {

                ESP_LOGW(TAG, "匹配成功");
                return "{\"结果\":\"成功\",\"播报\":\"已稳定识别到目标\"}";
            }

                elapsed += 100;
            }
        }

        ESP_LOGW(TAG, "6 次检测均失败");
        return "{\"结果\":\"失败\",\"播报\":\"检测超时，未识别到目标\"}";
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
    }
};

#endif // SJJC_H