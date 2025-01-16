// see also https://github.com/DaveDavenport/daly_bms_mqtt/tree/main

#pragma once

#include <stdint.h>

#define DALYBMS_MAX_MSG_LEN 14
#define DALYBMS_MAX_NUM_CELLS 24

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
        CMD_ID_BATTERY_FAILURE_STATE = 0x98
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

    /**
     * Structure for: CMD_ID_BMS_STATE
     */
    typedef struct
    {
        // charge/discharge (0 - stationary, 1 - charged, 2 - discharged )
        uint8_t status;
        // charge status
        uint8_t charge;
        // discharge status
        uint8_t discharge;
        //  bms cycle
        uint8_t life;
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
            dalybms_charge_state_t cs;
            dalybms_status_t status;
        };
    } dalybms_msg_t;


    dalybms_msg_t dalybms_read(const uart_port_t uart_num, dalybms_cmd_id_t cmd_id);
    dalybms_cell_voltages_t dalybms_read_cell_voltages(const uart_port_t uart_num, uint8_t num_cells);
    void dalybms_test(uart_port_t uart_num);

#ifdef __cplusplus
}
#endif
