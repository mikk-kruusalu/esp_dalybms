// see also https://github.com/DaveDavenport/daly_bms_mqtt/tree/main
// https://github.com/tizbac/daly-bms-uart-linux/tree/main
// and https://github.com/maland16/daly-bms-uart/tree/main

#pragma once

#include <stdint.h>
#include "driver/uart.h"

#define DALYBMS_MAX_MSG_LEN 13
#define DALYBMS_MAX_NUM_CELLS 24
#define DALYBMS_UART_TIMEOUT_MS 250
#define DALYBMS_MAX_HEADER_SEARCH 5
#define DALYBMS_DEFAULT_ADDRESS 0x80
#define DALYBMS_100BALANCE_ADDRESS 0x40

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * The different commands we can send.
     * These are taken from the manufacturer pdf.
     */
    typedef enum {
        // Total SOC, Voltage and current.
        CMD_ID_SOC_VOLTAGE_CURRENT = 0x90,
        // Minimum and maximum cell voltage and matching cell id.
        CMD_ID_MIN_MAX_CELL_VOLTAGE = 0x91,
        // Minimum and maximum temperature and matching cell id.
        CMD_ID_MIN_MAX_TEMPERATURE = 0x92,
        //
        CMD_ID_BMS_STATE = 0x93,
        //
        CMD_ID_STATUS = 0x94,
        // Voltages of each cell.
        CMD_ID_CELL_VOLTAGES = 0x95,
        // Temperature of each sensor.
        CMD_ID_TEMPERATURES = 0x96,
        //
        CMD_ID_CELL_BALANCE_STATE = 0x97,
        //
        CMD_ID_BATTERY_FAILURE_STATE = 0x98,
        //
        CMD_ID_DISCHARGE_FET = 0xD9,
        //
        CMD_ID_CHARGE_FET = 0xDA,
        //
        CMD_ID_BMS_RESET = 0x00,
    } dalybms_cmd_id_t;

    /**
     * Index into header.
     */
    typedef enum {
        CMD_INDEX_START = 0,
        CMD_INDEX_ADDRESS = 1,
        CMD_INDEX_DATA_ID = 2,
        CMD_INDEX_DATA_LEN = 3,
        CMD_INDEX_DATA = 4,
    } dalybms_msg_index_t;

    /**
     * Structures for each of the message type.
     */

    /**
     * Structure for: CMD_ID_SOC_VOLTAGE_CURRENT
     */
    typedef struct
    {
        // Voltage (V)
        float voltage;
        // Unsure
        float aquisition;
        // Current (A)
        float current;
        // State of charge (%)
        float soc;
    } dalybms_soc_voltage_current_t;

    /**
     * Structure for: CMD_ID_MIN_MAX_CELL_VOLTAGE
     */
    typedef struct
    {
        uint16_t max_mv;
        uint8_t max_id;
        uint16_t min_mv;
        uint8_t min_id;
        uint16_t cell_diff_mv;
    } dalybms_minmax_cellvoltage_t;

    /**
     * Structure for: CMD_ID_MIN_MAX_TEMPERATURE
     */
    typedef struct
    {
        int16_t max_temp;
        uint8_t max_id;
        int16_t min_temp;
        uint8_t min_id;
    } dalybms_minmax_temp_t;

    /**
     * Structure for: CMD_ID_CELL_VOLTAGES
     */
    typedef struct
    {
        uint8_t frame_num;
        uint16_t mvoltage[3];
    } dalybms_cell_voltage_frame_t;
    typedef struct
    {
        uint16_t mv[DALYBMS_MAX_NUM_CELLS];
    } dalybms_cell_voltages_t;

    /**
     * Structure for: CMD_ID_TEMPERATURES
     */
    typedef struct
    {
        uint8_t frame_num;
        uint16_t temp[4];
    } dalybms_temps_t;

    typedef struct
    {
        // 1 - open, 0 - closed
        uint32_t cells;
    } dalybms_balance_state_t;

    typedef enum {
        DALYBMS_STATE_STATIONARY = 0,
        DALYBMS_STATE_CHARGED,
        DALYBMS_STATE_DISCHARGED
    } dalybms_state_t;
    /**
     * Structure for: CMD_ID_BMS_STATE
     */
    typedef struct
    {
        // charge/discharge (0 - stationary, 1 - charged, 2 - discharged )
        dalybms_state_t status;
        // charge status
        uint8_t charge_mos;
        // discharge status
        uint8_t discharge_mos;
        //  bms cycle
        uint8_t ncycles;
        // residual capacity.
        uint32_t residual_charge;
    } dalybms_charge_state_t;

    /**
     * Structure for: CMD_ID_STATUS
     */
    typedef struct
    {
        // num_cells string?
        uint8_t num_cells;
        // temperature
        uint8_t num_temp;
        // charger status
        uint8_t charger;
        // load status
        uint8_t load;
        // cycle count.
        //uint16_t cycles;

    } dalybms_status_t;

    typedef enum
    {
        /* byte 0 */
        DALYBMS_FAIL_CELL_VOLTAGE_HIGH_1 = 0,
        DALYBMS_FAIL_CELL_VOLTAGE_HIGH_2,
        DALYBMS_FAIL_CELL_VOLTAGE_LOW_1,
        DALYBMS_FAIL_CELL_VOLTAGE_LOW_2,
        DALYBMS_FAIL_PACK_VOLTAGE_HIGH_1,
        DALYBMS_FAIL_PACK_VOLTAGE_HIGH_2,
        DALYBMS_FAIL_PACK_VOLTAGE_LOW_1,
        DALYBMS_FAIL_PACK_VOLTAGE_LOW_2,

        /* byte 1 */
        DALYBMS_FAIL_CHARGE_TEMP_HIGH_1 = 8,
        DALYBMS_FAIL_CHARGE_TEMP_HIGH_2,
        DALYBMS_FAIL_CHARGE_TEMP_LOW_1,
        DALYBMS_FAIL_CHARGE_TEMP_LOW_2,
        DALYBMS_FAIL_DISCHARGE_TEMP_HIGH_1,
        DALYBMS_FAIL_DISCHARGE_TEMP_HIGH_2,
        DALYBMS_FAIL_DISCHARGE_TEMP_LOW_1,
        DALYBMS_FAIL_DISCHARGE_TEMP_LOW_2,

        /* byte 2 */
        DALYBMS_FAIL_CHARGE_CURRENT_HIGH_1 = 16,
        DALYBMS_FAIL_CHARGE_CURRENT_HIGH_2,
        DALYBMS_FAIL_DISCHARGE_CURRENT_HIGH_1,
        DALYBMS_FAIL_DISCHARGE_CURRENT_HIGH_2,
        DALYBMS_FAIL_SOC_HIGH_1,
        DALYBMS_FAIL_SOC_HIGH_2,
        DALYBMS_FAIL_SOC_LOW_1,
        DALYBMS_FAIL_SOC_LOW_2,

        /* byte 3 */
        DALYBMS_FAIL_CELL_VOLTAGE_DIFFERENCE_HIGH_1 = 24,
        DALYBMS_FAIL_CELL_VOLTAGE_DIFFERENCE_HIGH_2,
        DALYBMS_FAIL_TEMP_SENSOR_DIFFERENCE_HIGH_1,
        DALYBMS_FAIL_TEMP_SENSOR_DIFFERENCE_HIGH_2,

        /* byte 4 */
        DALYBMS_FAIL_CHARGE_FET_TEMPERATURE_HIGH = 32,
        DALYBMS_FAIL_DISCHARGE_FET_TEMPERATURE_HIGH,
        DALYBMS_FAIL_CHARGE_FET_TEMPERATURE_SENSOR,
        DALYBMS_FAIL_DISCHARGE_FET_TEMPERATURE_SENSOR,
        DALYBMS_FAIL_CHARGE_FET_ADHESION,
        DALYBMS_FAIL_DISCHARGE_FET_ADHESION,
        DALYBMS_FAIL_CHARGE_FET_OPEN_CIRCUIT,
        DALYBMS_FAIL_DISCHARGE_FET_OPEN_CIRCUIT,

        /* byte 5 */
        DALYBMS_FAIL_AFE_ACQUISITION_MODULE = 36,
        DALYBMS_FAIL_VOLTAGE_SENSOR_MODULE,
        DALYBMS_FAIL_TEMPERATURE_SENSOR_MODULE,
        DALYBMS_FAIL_EEPROM_MODULE,
        DALYBMS_FAIL_RTC_MODULE,
        DALYBMS_FAIL_PRECHARGE_MODULE,
        DALYBMS_FAIL_COMMUNICATION_MODULE,
        DALYBMS_FAIL_INTERNAL_COMMUNICATION_MODULE,

        /* byte 6 */
        DALYBMS_FAIL_CURRENT_SENSOR_MODULE = 42,
        DALYBMS_FAIL_MAIN_VOLTAGE_SENSOR_MODULE,
        DALYBMS_FAIL_SHORT_CIRCUIT_PROTECTION,
        DALYBMS_FAIL_LOW_VOLTAGE_NO_CHARGING,
    } dalybms_failure_code_t;

    typedef struct {
        uint8_t bitmask[7];
        uint8_t code;
    } dalybms_failure_t;

    /**
     * A parent structure for daly message.
     * Useful for passing messages via a queue.
     */
    typedef struct
    {
        // Type of message this structure hold.
        dalybms_cmd_id_t id;
        union
        {
            dalybms_soc_voltage_current_t soc;
            dalybms_minmax_cellvoltage_t mmcv;
            dalybms_minmax_temp_t mmt;
            dalybms_cell_voltage_frame_t cvf;
            dalybms_temps_t tmps;
            dalybms_balance_state_t bs;
            dalybms_charge_state_t cs;
            dalybms_status_t status;
            dalybms_failure_t fail;
        };
        esp_err_t error;
    } dalybms_msg_t;

    typedef struct
    {
        uart_port_t uart_num;
        uint8_t address;
    } dalybms_t;

    dalybms_msg_t dalybms_read(const dalybms_t *bms, dalybms_cmd_id_t cmd_id);
    dalybms_cell_voltages_t dalybms_read_cell_voltages(const dalybms_t *bms, uint8_t num_cells);
    void dalybms_test(const dalybms_t *bms);
    void dalybms_set_charge_fet(const dalybms_t *bms, uint8_t level);
    void dalybms_set_discharge_fet(const dalybms_t *bms, uint8_t level);
    void dalybms_reset(const dalybms_t *bms);
    bool dalybms_is_failure(dalybms_failure_t failure, dalybms_failure_code_t failure_code);

#ifdef __cplusplus
}
#endif
