#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "dalybms.h"

static const char *TAG = "dalybms";

static uint8_t _dalybms_calc_checksum(const uint8_t *msg)
{
    uint8_t chk = 0;
    chk += msg[0];
    chk += msg[1];
    chk += msg[2];
    chk += msg[3];
    for (uint8_t iter = 0; iter < msg[3]; iter++)
    {
        chk += msg[4 + iter];
    }
    return chk;
}

static void _dalybms_send_command(
    const dalybms_cmd_id_t cmdid,
    const uart_port_t uart_num,
    const uint8_t *data
)
{
    ESP_LOGI(TAG, "Sending command %#02x", cmdid);
    // With BMS we send a command, and wait for reply.
    uint8_t command_msg[DALYBMS_MAX_MSG_LEN] = {
        0xA5,  // 0 Start byte
        0x40,  // 1 'Upper' module
        cmdid, // 2 Command byte.
        0x08,  // 3 Length
        0x00,  // 4 data 0
        0x00,  // 5 data 1
        0x00,  // 6 data 2
        0x00,  // 7 data 3
        0x00,  // 8 data 4
        0x00,  // 9 data 5
        0x00,  // 10 data 6
        0x00,  // 11 data 7
        0x00   // 12 Checksum
    };

    if (data != NULL) {
        memcpy(command_msg+4, data, 8);
    }
    // Calculate the checksum
    command_msg[DALYBMS_MAX_MSG_LEN - 1] = _dalybms_calc_checksum(command_msg);

    // Send it to the bms.
    uart_write_bytes(uart_num, (const char *)command_msg, DALYBMS_MAX_MSG_LEN);
}

static dalybms_msg_t _dalybms_process_msg(const uint8_t *msg)
{
    dalybms_msg_t ret_msg = {
        0,
    };
    ret_msg.id = (dalybms_cmd_id_t)(msg[CMD_INDEX_DATA_ID]);

    switch (ret_msg.id)
    {
        case CMD_ID_SOC_VOLTAGE_CURRENT: {
            ret_msg.soc.voltage = ((float)((msg[4] << 8) | msg[5]) / 10.0f);
            ret_msg.soc.aquisition = ((float)((msg[6] << 8) | msg[7]) / 10.0f);
            ret_msg.soc.current = (
                (float)(((msg[8] << 8) | msg[9]) - 30000) / 10.0f
            );
            ret_msg.soc.soc = ((float)((msg[10] << 8) | msg[11]) / 10.0f);
            ESP_LOGI(
                TAG,
                "SOC: %.2f %%, %.2f V, %.2f A",
                ret_msg.soc.soc,
                ret_msg.soc.voltage,
                ret_msg.soc.current
            );
            break;
        }
        case CMD_ID_MIN_MAX_CELL_VOLTAGE: {
            ret_msg.mmcv.max_mv = (msg[4] << 8 | msg[5]);
            ret_msg.mmcv.max_id = (msg[6]);
            ret_msg.mmcv.min_mv = (msg[7] << 8 | msg[8]);
            ret_msg.mmcv.min_id = (msg[9]);
            ret_msg.mmcv.cell_diff_mv = (
                ret_msg.mmcv.max_mv - ret_msg.mmcv.min_mv
            );
            ESP_LOGI(
                TAG,
                "Max: %d mV (%d)  Min: %d mV (%d)  diff: %d mV",
                ret_msg.mmcv.max_mv,
                ret_msg.mmcv.max_id,
                ret_msg.mmcv.min_mv,
                ret_msg.mmcv.min_id,
                ret_msg.mmcv.cell_diff_mv
            );
            break;
        }
        case CMD_ID_MIN_MAX_TEMPERATURE: {
            ret_msg.mmt.max_temp = msg[4] - 40;
            ret_msg.mmt.max_id = msg[5];
            ret_msg.mmt.min_temp = msg[6] - 40;
            ret_msg.mmt.min_id = msg[7];
            ESP_LOGI(
                TAG,
                "Max: %i C (%d)  Min: %i C (%d)",
                ret_msg.mmt.max_temp,
                ret_msg.mmt.max_id,
                ret_msg.mmt.min_temp,
                ret_msg.mmt.min_id
            );
            break;
        }
        case CMD_ID_BMS_STATE: {
            ret_msg.cs.status = msg[4];
            ret_msg.cs.charge = msg[5];
            ret_msg.cs.discharge = msg[6];
            ret_msg.cs.ncycles = msg[7];
            ret_msg.cs.residual_charge = (
                msg[8] << 24 | msg[9] << 16 | msg[10] << 8 | msg[11]
            );
            ESP_LOGI(
                TAG,
                "State: %d  Charge MOS: %d  Discharge MOS: %d Cycles: %d"
                "  Remaining Capacity: %ld mAh",
                ret_msg.cs.status,
                ret_msg.cs.charge,
                ret_msg.cs.discharge,
                ret_msg.cs.ncycles,
                ret_msg.cs.residual_charge
            );
            break;
        }
        case CMD_ID_STATUS: {
            ret_msg.status.num_cells = msg[4];
            ret_msg.status.num_temp = msg[5];
            ret_msg.status.charger = msg[6];
            ret_msg.status.load = msg[7];
            // msg[8] skip, no idea what it is.
            //ret_msg.status.cycles = msg[9] << 8 | msg[10];
            ESP_LOGI(
                TAG,
                "Number of cells: %d  Number of NTC: %d  Charger: %d  Load: %d",
                ret_msg.status.num_cells,
                ret_msg.status.num_temp,
                ret_msg.status.charger,
                ret_msg.status.load
            );
            break;
        }
        case CMD_ID_CELL_VOLTAGES: {
            ret_msg.cvf.frame_num = msg[4];
            for (uint8_t i = 0; i < 3; i++) {
                ret_msg.cvf.mvoltage[i] = (msg[5 + 2*i] << 8 | msg[6 + 2*i]);
            }

            ESP_LOGI(
                TAG,
                "Frame %d  1: %d mV  2: %d mV  3: %d mV",
                ret_msg.cvf.frame_num,
                ret_msg.cvf.mvoltage[0],
                ret_msg.cvf.mvoltage[1],
                ret_msg.cvf.mvoltage[2]
            );
            break;
        }
        case CMD_ID_TEMPERATURES: {
            ret_msg.tmps.frame_num = msg[4];
            ret_msg.tmps.temp[0] = msg[5] - 40;
            ret_msg.tmps.temp[1] = msg[6] - 40;
            ret_msg.tmps.temp[2] = msg[7] - 40;
            ret_msg.tmps.temp[3] = msg[8] - 40;
            ESP_LOGI(
                TAG,
                "Frame %d  1: %d C  2: %d C  3: %d C  4: %d C",
                ret_msg.tmps.frame_num,
                ret_msg.tmps.temp[0],
                ret_msg.tmps.temp[1],
                ret_msg.tmps.temp[2],
                ret_msg.tmps.temp[3]
            );
            break;
        }
        case CMD_ID_CELL_BALANCE_STATE: {
            ret_msg.bs.cells = (
                msg[7] << 24 | msg[6] << 16 | msg[5] << 8 | msg[4]
            );
            ESP_LOGI(
                TAG,
                "Cell balance state %#04lX",
                ret_msg.bs.cells
            );
            break;
        }
        case CMD_ID_DISCHARGE_FET: {
            ESP_LOGI(TAG, "Discharge MOS set to level %d", msg[4]);
            break;
        }
        case CMD_ID_CHARGE_FET: {
            ESP_LOGI(TAG, "Charge MOS set to level %d", msg[4]);
            break;
        }
        case CMD_ID_BATTERY_FAILURE_STATE: {
            ret_msg.fail.code = msg[7];
            ESP_LOGI(
                TAG,
                "Got failure code %d",
                ret_msg.fail.code
            );
            for (uint8_t i = 0; i < 7; i++) {
                ret_msg.fail.bitmask[i] = msg[i + 4];
                ESP_LOGI(TAG, "Byte %d: %#02X", i, msg[i + 4]);
            }
            break;
        }
        default:
            ESP_LOGW(TAG, "Unknown command %#02X", ret_msg.id);
    }

    return ret_msg;
}

static esp_err_t _uart_read_byte(
    uart_port_t uart_num, void *buf, uint32_t length
)
{
    int uart_ret = uart_read_bytes(uart_num, buf, length, pdMS_TO_TICKS(250));
    if (uart_ret == 0) {
        // Timeout
        ESP_LOGE(TAG, "Error, uart timeout");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void _dalybms_read_response(
    const uart_port_t uart_num, uint8_t *raw_msg
)
{
    uint8_t c = 0x0;

    // Search for header.
    while (c != 0xA5) {
        _uart_read_byte(uart_num, &c, 1);
    }
    raw_msg[CMD_INDEX_START] = c;

    // get address
    _uart_read_byte(uart_num, &c, 1);
    raw_msg[CMD_INDEX_ADDRESS] = c;

    // get data id
    _uart_read_byte(uart_num, &c, 1);
    raw_msg[CMD_INDEX_DATA_ID] = c;

    // get data length
    _uart_read_byte(uart_num, &c, 1);
    raw_msg[CMD_INDEX_DATA_LEN] = c;

    // iterate over data
    for (uint8_t iter = 0; iter < raw_msg[CMD_INDEX_DATA_LEN]; iter++) {
        _uart_read_byte(uart_num, &c, 1);
        raw_msg[4 + iter] = c;
    }

    // get checksum
    _uart_read_byte(uart_num, &c, 1);
    raw_msg[12] = c;

    uint8_t checksum = _dalybms_calc_checksum(raw_msg);
    if (c != checksum) {
        ESP_LOGE(TAG, "Error, checksum failed: %02X - %02X (%d)", c, checksum,
                    raw_msg[CMD_INDEX_DATA_LEN]);
        for (int i = 0; i < DALYBMS_MAX_MSG_LEN; i++) {
            ESP_LOGE(TAG, "%d: %02X", i, raw_msg[i]);
        }
        return;
    }

    for (int i = 0; i < DALYBMS_MAX_MSG_LEN; i++) {
        ESP_LOGD(TAG, "%d: %02X", i, raw_msg[i]);
    }
}

dalybms_msg_t dalybms_read(const uart_port_t uart_num, dalybms_cmd_id_t cmd_id)
{
    _dalybms_send_command(cmd_id, uart_num, NULL);

    uint8_t raw_msg[DALYBMS_MAX_MSG_LEN] = {
        0x00,
    };
    _dalybms_read_response(uart_num, raw_msg);
    dalybms_msg_t msg = _dalybms_process_msg(raw_msg);
    return msg;
}

dalybms_cell_voltages_t dalybms_read_cell_voltages(
    const uart_port_t uart_num, uint8_t num_cells
)
{
    uint8_t raw_msg[DALYBMS_MAX_MSG_LEN] = {
        0x00,
    };
    dalybms_cell_voltages_t cvs;

    dalybms_msg_t frame_msg = dalybms_read(uart_num, CMD_ID_CELL_VOLTAGES);
    for (uint8_t i = 0; i < 3; i++) {
        cvs.mv[i] = frame_msg.cvf.mvoltage[i];
    }

    uint8_t num_frames = ceil(num_cells / 3.0);
    for (uint8_t i = 0; i < num_frames-1; i++) {
        _dalybms_read_response(uart_num, raw_msg);
        frame_msg = _dalybms_process_msg(raw_msg);

        uint8_t offset = (frame_msg.cvf.frame_num - 1) * 3;
        for (uint8_t i = 0; i < 3; i++) {
            cvs.mv[i + offset] = frame_msg.cvf.mvoltage[i];
        }
    }

    for (uint8_t i = 0; i < num_cells; i++) {
        ESP_LOGI(
            TAG,
            "Cell %d: %d mV",
            i,
            cvs.mv[i]
        );
    }

    return cvs;
}

static void _dalybms_set_fet(
    uart_port_t uart_num, dalybms_cmd_id_t cmd_id, uint8_t level
)
{
    uint8_t data[8] = {
        level,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };
    _dalybms_send_command(cmd_id, uart_num, data);

    uint8_t raw_msg[DALYBMS_MAX_MSG_LEN] = {
        0x00,
    };
    _dalybms_read_response(uart_num, raw_msg);
    _dalybms_process_msg(raw_msg);
}

void dalybms_set_discharge_fet(uart_port_t uart_num, uint8_t level)
{
    _dalybms_set_fet(uart_num, CMD_ID_DISCHARGE_FET, level);
}

void dalybms_set_charge_fet(uart_port_t uart_num, uint8_t level)
{
    _dalybms_set_fet(uart_num, CMD_ID_CHARGE_FET, level);
}

void dalybms_reset(uart_port_t uart_num)
{
    ESP_LOGI(TAG, "Send BMS reset command");
    _dalybms_send_command(CMD_ID_BMS_RESET, uart_num, NULL);
}

bool dalybms_is_failure(
    dalybms_failure_t failure,
    dalybms_failure_code_t failure_code
)
{
    uint8_t nbyte = floor(failure_code / 8.0);
    uint8_t bitmask = 1 << (failure_code - nbyte*8);

    return failure.bitmask[nbyte] & bitmask ? true : false;
}

void dalybms_test(uart_port_t uart_num)
{
    ESP_LOGI(TAG, "Testing BMS connection");
    dalybms_read(uart_num, CMD_ID_SOC_VOLTAGE_CURRENT);
    dalybms_read(uart_num, CMD_ID_MIN_MAX_CELL_VOLTAGE);
    dalybms_read(uart_num, CMD_ID_MIN_MAX_TEMPERATURE);
    dalybms_read(uart_num, CMD_ID_TEMPERATURES);
    dalybms_read(uart_num, CMD_ID_CELL_BALANCE_STATE);
    dalybms_read(uart_num, CMD_ID_BMS_STATE);
    dalybms_read(uart_num, CMD_ID_STATUS);
    dalybms_read(uart_num, CMD_ID_BATTERY_FAILURE_STATE);
    dalybms_read_cell_voltages(uart_num, 8);
    ESP_LOGI(TAG, "BMS connection test done.");
}
