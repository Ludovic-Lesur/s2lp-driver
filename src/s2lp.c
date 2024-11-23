/*
 * s2lp.c
 *
 *  Created on: 17 nov. 2024
 *      Author: Ludo
 */

#include "s2lp.h"

#ifndef S2LP_DRIVER_DISABLE_FLAGS_FILE
#include "s2lp_driver_flags.h"
#endif
#include "s2lp_hw.h"
#include "types.h"

#ifndef S2LP_DRIVER_DISABLE

/*** S2LP local macros ***/

// SPI header bytes.
#define S2LP_HEADER_BYTE_WRITE                  0x00
#define S2LP_HEADER_BYTE_READ                   0x01
#define S2LP_HEADER_BYTE_COMMAND                0x80
// Transfer size.
#define S2LP_REGISTER_SPI_TRANSFER_SIZE         3
#define S2LP_COMMAND_SPI_TRANSFER_SIZE          2
#define S2LP_FIFO_SPI_TRANSFER_SIZE             2
// Crystal frequency ranges.
#define S2LP_XO_HIGH_RANGE_THRESHOLD_HZ         48000000
// Digital frequency.
#if (S2LP_DRIVER_XO_FREQUENCY_HZ >= S2LP_XO_HIGH_RANGE_THRESHOLD_HZ)
#define S2LP_PD_CLKDIB_BIT                      0
#define S2LP_PLL_PFD_SPLIT_EN_BIT               0
#define S2LP_FDIG_HZ                            (S2LP_DRIVER_XO_FREQUENCY_HZ >> 1)
#define S2LP_IF_OFFSET_ANA                      0x2A
#else
#define S2LP_PD_CLKDIB_BIT                      1
#define S2LP_PLL_PFD_SPLIT_EN_BIT               1
#define S2LP_FDIG_HZ                            S2LP_DRIVER_XO_FREQUENCY_HZ
#define S2LP_IF_OFFSET_ANA                      0xB8
#endif
// RF frequency range.
#define S2LP_RF_FREQUENCY_HZ_MIN                826000000
#define S2LP_RF_FREQUENCY_HZ_MAX                958000000
// Frequency deviation range.
#define S2LP_DEVIATION_HZ_MIN                   150
#define S2LP_DEVIATION_HZ_MAX                   500000
#define S2LP_DEVIATION_MANTISSA_MAX             255
#define S2LP_DEVIATION_MANTISSA_OFFSET          256
#define S2LP_DEVIATION_EXPONENT_MAX             15
// RX bandwidth range.
#define S2LP_RX_BANDWIDTH_HZ_MIN                1100
#define S2LP_RX_BANDWIDTH_HZ_MAX                800100
#define S2LP_RX_BANDWIDTH_TABLE_SIZE            90
#define S2LP_RX_BANDWIDTH_TABLE_FREQUENCY_KHZ   260000
// Datarate range.
#define S2LP_DATARATE_BPS_MIN                   100
#define S2LP_DATARATE_BPS_MAX                   125000
#define S2LP_DATARATE_MANTISSA_MAX              65535
#define S2LP_DATARATE_MANTISSA_OFFSET           65536
#define S2LP_DATARATE_EXPONENT_MAX              14
// RF output power range.
#define S2LP_RF_OUTPUT_POWER_MIN                -49
#define S2LP_RF_OUTPUT_POWER_MAX                14
// Sync word max length.
#define S2LP_SYNC_WORD_LENGTH_BITS_MAX          32
// RSSI offset.
#define S2LP_RSSI_OFFSET_DB                     146
// FIFO.
#define S2LP_FIFO_THHRESHOLD_BYTES_MAX          0x7F
// Timeout.
#define S2LP_TIMEOUT_COUNT                      100000000

/*** S2LP local structures ***/

/*******************************************************************/
typedef enum {
    S2LP_REGISTER_GPIO0_CONF = 0x00,
    S2LP_REGISTER_GPIO1_CONF = 0x01,
    S2LP_REGISTER_GPIO2_CONF = 0x02,
    S2LP_REGISTER_GPIO3_CONF = 0x03,
    S2LP_REGISTER_SYNT3 = 0x05,
    S2LP_REGISTER_SYNT2 = 0x06,
    S2LP_REGISTER_SYNT1 = 0x07,
    S2LP_REGISTER_SYNT0 = 0x08,
    S2LP_REGISTER_IF_OFFSET_ANA = 0x09,
    S2LP_REGISTER_IF_OFFSET_DIG = 0x0A,
    S2LP_REGISTER_CHSPACE = 0x0C,
    S2LP_REGISTER_CHNUM = 0x0D,
    S2LP_REGISTER_MOD4 = 0x0E,
    S2LP_REGISTER_MOD3 = 0x0F,
    S2LP_REGISTER_MOD2 = 0x10,
    S2LP_REGISTER_MOD1 = 0x11,
    S2LP_REGISTER_MOD0 = 0x12,
    S2LP_REGISTER_CHFLT = 0x13,
    S2LP_REGISTER_AFC2 = 0x14,
    S2LP_REGISTER_AFC1 = 0x15,
    S2LP_REGISTER_AFC0 = 0x16,
    S2LP_REGISTER_RSSI_FLT = 0x17,
    S2LP_REGISTER_RSSI_TH = 0x18,
    S2LP_REGISTER_AGCCTRL4 = 0x1A,
    S2LP_REGISTER_AGCCTRL3 = 0x1B,
    S2LP_REGISTER_AGCCTRL2 = 0x1C,
    S2LP_REGISTER_AGCCTRL1 = 0x1D,
    S2LP_REGISTER_AGCCTRL0 = 0x1E,
    S2LP_REGISTER_ANT_SELECT_CONF = 0x1F,
    S2LP_REGISTER_CLOCKREC2 = 0x20,
    S2LP_REGISTER_CLOCKREC1 = 0x21,
    S2LP_REGISTER_PCKTCTRL6 = 0x2B,
    S2LP_REGISTER_PCKTCTRL5 = 0x2C,
    S2LP_REGISTER_PCKTCTRL4 = 0x2D,
    S2LP_REGISTER_PCKTCTRL3 = 0x2E,
    S2LP_REGISTER_PCKTCTRL2 = 0x2F,
    S2LP_REGISTER_PCKTCTRL1 = 0x30,
    S2LP_REGISTER_PCKTLEN1 = 0x31,
    S2LP_REGISTER_PCKTLEN0 = 0x32,
    S2LP_REGISTER_SYNC3 = 0x33,
    S2LP_REGISTER_SYNC2 = 0x34,
    S2LP_REGISTER_SYNC1 = 0x35,
    S2LP_REGISTER_SYNC0 = 0x36,
    S2LP_REGISTER_QI = 0x37,
    S2LP_REGISTER_PCKT_PSTMLB = 0x38,
    S2LP_REGISTER_PROTOCOL2 = 0x39,
    S2LP_REGISTER_PROTOCOL1 = 0x3A,
    S2LP_REGISTER_PROTOCOL0 = 0x3B,
    S2LP_REGISTER_FIFO_CONFIG3 = 0x3C,
    S2LP_REGISTER_FIFO_CONFIG2 = 0x3D,
    S2LP_REGISTER_FIFO_CONFIG1 = 0x3E,
    S2LP_REGISTER_FIFO_CONFIG0 = 0x3F,
    S2LP_REGISTER_PCKT_FLT_OPTIONS = 0x40,
    S2LP_REGISTER_PCKT_FLT_GOALS4 = 0x41,
    S2LP_REGISTER_PCKT_FLT_GOALS3 = 0x42,
    S2LP_REGISTER_PCKT_FLT_GOALS2 = 0x43,
    S2LP_REGISTER_PCKT_FLT_GOALS1 = 0x44,
    S2LP_REGISTER_PCKT_FLT_GOALS0 = 0x45,
    S2LP_REGISTER_TIMERS5 = 0x46,
    S2LP_REGISTER_TIMERS4 = 0x47,
    S2LP_REGISTER_TIMERS3 = 0x48,
    S2LP_REGISTER_TIMERS2 = 0x49,
    S2LP_REGISTER_TIMERS1 = 0x4A,
    S2LP_REGISTER_TIMERS0 = 0x4B,
    S2LP_REGISTER_CSMA_CONF3 = 0x4C,
    S2LP_REGISTER_CSMA_CONF2 = 0x4D,
    S2LP_REGISTER_CSMA_CONF1 = 0x4E,
    S2LP_REGISTER_CSMA_CONF0 = 0x4F,
    S2LP_REGISTER_IRQ_MASK3 = 0x50,
    S2LP_REGISTER_IRQ_MASK2 = 0x51,
    S2LP_REGISTER_IRQ_MASK1 = 0x52,
    S2LP_REGISTER_IRQ_MASK0 = 0x53,
    S2LP_REGISTER_FAST_RX_TIMER = 0x54,
    S2LP_REGISTER_PA_POWER8 = 0x5A,
    S2LP_REGISTER_PA_POWER7 = 0x5B,
    S2LP_REGISTER_PA_POWER6 = 0x5C,
    S2LP_REGISTER_PA_POWER5 = 0x5D,
    S2LP_REGISTER_PA_POWER4 = 0x5E,
    S2LP_REGISTER_PA_POWER3 = 0x5F,
    S2LP_REGISTER_PA_POWER2 = 0x60,
    S2LP_REGISTER_PA_POWER1 = 0x61,
    S2LP_REGISTER_PA_POWER0 = 0x62,
    S2LP_REGISTER_PA_CONFIG1 = 0x63,
    S2LP_REGISTER_PA_CONFIG0 = 0x64,
    S2LP_REGISTER_SYNTH_CONFIG2 = 0x65,
    S2LP_REGISTER_VCO_CONFIG = 0x68,
    S2LP_REGISTER_VCO_CALIBR_IN2 = 0x69,
    S2LP_REGISTER_VCO_CALIBR_IN1 = 0x6A,
    S2LP_REGISTER_VCO_CALIBR_IN0 = 0x6B,
    S2LP_REGISTER_XO_RCO_CONF1 = 0x6C,
    S2LP_REGISTER_XO_RCO_CONF0 = 0x6D,
    S2LP_REGISTER_RCO_CALIBR_CONF3 = 0x6E,
    S2LP_REGISTER_RCO_CALIBR_CONF2 = 0x6F,
    S2LP_REGISTER_PM_CONF4 = 0x75,
    S2LP_REGISTER_PM_CONF3 = 0x76,
    S2LP_REGISTER_PM_CONF2 = 0x77,
    S2LP_REGISTER_PM_CONF1 = 0x78,
    S2LP_REGISTER_PM_CONF0 = 0x79,
    S2LP_REGISTER_MC_STATE1 = 0x8D,
    S2LP_REGISTER_MC_STATE0 = 0x8E,
    S2LP_REGISTER_TX_FIFO_STATUS = 0x8F,
    S2LP_REGISTER_RX_FIFO_STATUS = 0x90,
    S2LP_REGISTER_RCO_CALIBR_OUT4 = 0x94,
    S2LP_REGISTER_RCO_CALIBR_OUT3 = 0x95,
    S2LP_REGISTER_VCO_CALIBR_OUT1 = 0x99,
    S2LP_REGISTER_VCO_CALIBR_OUT0 = 0x9A,
    S2LP_REGISTER_TX_PCKT_INFO = 0x9C,
    S2LP_REGISTER_RX_PCKT_INFO = 0x9D,
    S2LP_REGISTER_AFC_CORR = 0x9E,
    S2LP_REGISTER_LINK_QUALIF2 = 0x9F,
    S2LP_REGISTER_LINK_QUALIF1 = 0xA0,
    S2LP_REGISTER_RSSI_LEVEL = 0xA2,
    S2LP_REGISTER_RX_PCKT_LEN1 = 0xA4,
    S2LP_REGISTER_RX_PCKT_LEN0 = 0xA5,
    S2LP_REGISTER_CRC_FIELD3 = 0xA6,
    S2LP_REGISTER_CRC_FIELD2 = 0xA7,
    S2LP_REGISTER_CRC_FIELD1 = 0xA8,
    S2LP_REGISTER_CRC_FIELD0 = 0xA9,
    S2LP_REGISTER_RX_ADDRE_FIELD1 = 0xAA,
    S2LP_REGISTER_RX_ADDRE_FIELD0 = 0xAB,
    S2LP_REGISTER_RSSI_LEVEL_RUN = 0xEF,
    S2LP_REGISTER_DEVICE_INFO1 = 0xF0,
    S2LP_REGISTER_DEVICE_INFO0 = 0xF1,
    S2LP_REGISTER_IRQ_STATUS3 = 0xFA,
    S2LP_REGISTER_IRQ_STATUS2 = 0xFB,
    S2LP_REGISTER_IRQ_STATUS1 = 0xFC,
    S2LP_REGISTER_IRQ_STATUS0 = 0xFD,
    S2LP_REGISTER_FIFO = 0xFF
} S2LP_register_t;

/*******************************************************************/
typedef struct {
    uint16_t mantissa;
    uint8_t exponent;
} S2LP_mantissa_exponent_t;

/*** S2LP local global variables ***/

#ifdef S2LP_DRIVER_RX_ENABLE
static const uint16_t S2LP_RX_BANDWIDTH_26M[S2LP_RX_BANDWIDTH_TABLE_SIZE] = {
    8001, 7951, 7684, 7368, 7051, 6709, 6423, 5867, 5414,
    4509, 4259, 4032, 3808, 3621, 3417, 3254, 2945, 2703,
    2247, 2124, 2015, 1900, 1807, 1706, 1624, 1471, 1350,
    1123, 1062, 1005, 950,  903,  853,  812,  735,  675,
    561,  530,  502,  474,  451,  426,  406,  367,  337,
    280,  265,  251,  237,  226,  213,  203,  184,  169,
    140,  133,  126,  119,  113,  106,  101,  92,   84,
    70,   66,   63,   59,   56,   53,   51,   46,   42,
    35,   33,   31,   30,   28,   27,   25,   23,   21,
    18,   17,   16,   15,   14,   13,   13,   12,   11
};
#endif

/*** S2LP local functions ***/

/*******************************************************************/
static S2LP_status_t _S2LP_write_register(uint8_t addr, uint8_t value) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t tx_data[S2LP_REGISTER_SPI_TRANSFER_SIZE];
    uint8_t rx_data[S2LP_REGISTER_SPI_TRANSFER_SIZE];
    // Write sequence.
    tx_data[0] = S2LP_HEADER_BYTE_WRITE;
    tx_data[1] = addr;
    tx_data[2] = value;
    status = S2LP_HW_spi_write_read_8(tx_data, rx_data, S2LP_REGISTER_SPI_TRANSFER_SIZE);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static S2LP_status_t _S2LP_read_register(uint8_t addr, uint8_t* value) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t tx_data[S2LP_REGISTER_SPI_TRANSFER_SIZE];
    uint8_t rx_data[S2LP_REGISTER_SPI_TRANSFER_SIZE];
    // Read sequence.
    tx_data[0] = S2LP_HEADER_BYTE_READ;
    tx_data[1] = addr;
    tx_data[2] = 0xFF;
    // Read sequence.
    status = S2LP_HW_spi_write_read_8(tx_data, rx_data, S2LP_REGISTER_SPI_TRANSFER_SIZE);
    if (status != S2LP_SUCCESS) goto errors;
    // Read value.
    (*value) = rx_data[2];
errors:
    return status;
}

/*******************************************************************/
static uint32_t _S2LP_compute_datarate(S2LP_mantissa_exponent_t* datarate_setting) {
    // Local variables.
    uint64_t tmp_u64 = 0;
    uint32_t datarate_bps = 0;
    // Compute formula.
    if ((datarate_setting->exponent) == 0) {
        tmp_u64 = (uint64_t) S2LP_FDIG_HZ * (uint64_t) (datarate_setting->mantissa);
        datarate_bps = (uint32_t) (tmp_u64 >> 32);
    }
    else {
        tmp_u64 = ((uint64_t) S2LP_FDIG_HZ) * (((uint64_t) (datarate_setting->mantissa)) + S2LP_DATARATE_MANTISSA_OFFSET);
        datarate_bps = (uint32_t) (tmp_u64 >> (33 - (datarate_setting->exponent)));
    }
    return datarate_bps;
}

/*******************************************************************/
static S2LP_status_t _S2LP_compute_mantissa_exponent_datarate(uint32_t datarate_bps, S2LP_mantissa_exponent_t* datarate_setting) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    S2LP_mantissa_exponent_t tmp_setting;
    uint32_t datarate_max = 0;
    uint8_t e = 0;
    uint64_t tmp_u64 = 0;
    // Check range.
    if (datarate_bps < S2LP_DATARATE_BPS_MIN) {
        status = S2LP_ERROR_DATARATE_UNDERFLOW;
        goto errors;
    }
    if (datarate_bps > S2LP_DATARATE_BPS_MAX) {
        status = S2LP_ERROR_DATARATE_OVERFLOW;
        goto errors;
    }
    // Select exponent.
    for (e = 0; e < S2LP_DATARATE_EXPONENT_MAX; e++) {
        // Compute maximum value.
        tmp_setting.mantissa = S2LP_DATARATE_MANTISSA_MAX;
        tmp_setting.exponent = e;
        datarate_max = _S2LP_compute_datarate(&tmp_setting);
        // Check range.
        if (datarate_bps <= datarate_max) break;
    }
    (datarate_setting->exponent) = e;
    // Compute mantissa.
    if (e == 0) {
        tmp_u64 = ((uint64_t) datarate_bps << 32);
        (datarate_setting->mantissa) = ((tmp_u64) / ((uint64_t) S2LP_FDIG_HZ));
    }
    else {
        tmp_u64 = ((uint64_t) datarate_bps << (33 - e));
        (datarate_setting->mantissa) = ((tmp_u64) / ((uint64_t) S2LP_FDIG_HZ)) - S2LP_DATARATE_MANTISSA_OFFSET;
    }
errors:
    return status;
}

/*******************************************************************/
static uint32_t _S2LP_compute_deviation(S2LP_mantissa_exponent_t* deviation_setting) {
    // Local variables.
    uint32_t deviation_hz = 0;
    uint64_t tmp_u64 = 0;
    // Check exponent.
    if ((deviation_setting->exponent) == 0) {
        tmp_u64 = (uint64_t) S2LP_DRIVER_XO_FREQUENCY_HZ * (uint64_t) (deviation_setting->mantissa);
        deviation_hz = (uint32_t) (tmp_u64 >> 22);
    }
    else {
        tmp_u64 = (uint64_t) S2LP_DRIVER_XO_FREQUENCY_HZ * (uint64_t) ((deviation_setting->mantissa) + S2LP_DEVIATION_MANTISSA_OFFSET);
        deviation_hz = (uint32_t) (tmp_u64 >> (23 - (deviation_setting->exponent)));
    }
    return deviation_hz;
}

/*******************************************************************/
static S2LP_status_t _S2LP_compute_mantissa_exponent_deviation(uint32_t deviation_hz, S2LP_mantissa_exponent_t* deviation_setting) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    S2LP_mantissa_exponent_t tmp_setting;
    uint32_t deviation_max = 0;
    uint8_t e = 0;
    uint64_t tmp_u64 = 0;
    // Check range.
    if (deviation_hz < S2LP_DEVIATION_HZ_MIN) {
        status = S2LP_ERROR_DEVIATION_UNDERFLOW;
        goto errors;
    }
    if (deviation_hz > S2LP_DEVIATION_HZ_MAX) {
        status = S2LP_ERROR_DEVIATION_OVERFLOW;
        goto errors;
    }
    // Select exponent.
    for (e = 0; e < S2LP_DEVIATION_EXPONENT_MAX; e++) {
        // Compute maximum value.
        tmp_setting.mantissa = S2LP_DEVIATION_MANTISSA_MAX;
        tmp_setting.exponent = e;
        deviation_max = _S2LP_compute_deviation(&tmp_setting);
        // Check range.
        if (deviation_hz <= deviation_max) break;
    }
    (deviation_setting->exponent) = e;
    // Compute mantissa.
    if (e == 0) {
        tmp_u64 = ((uint64_t) deviation_hz << 22);
        (deviation_setting->mantissa) = ((tmp_u64) / ((uint64_t) S2LP_DRIVER_XO_FREQUENCY_HZ));
    }
    else {
        tmp_u64 = ((uint64_t) deviation_hz << (23 - e));
        (deviation_setting->mantissa) = ((tmp_u64) / ((uint64_t) S2LP_DRIVER_XO_FREQUENCY_HZ)) - S2LP_DEVIATION_MANTISSA_OFFSET;
    }
errors:
    return status;
}

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
static uint32_t _S2LP_compute_rx_bandwidth(uint8_t rx_bandwidth_table_index) {
    // Local variables.
    uint32_t rx_bandwidth_hz = 0;
    uint64_t tmp_u64 = 0;
    // Check index.
    if (rx_bandwidth_table_index < S2LP_RX_BANDWIDTH_TABLE_SIZE) {
        tmp_u64 = ((uint64_t) S2LP_RX_BANDWIDTH_26M[rx_bandwidth_table_index]) * ((uint64_t) S2LP_FDIG_HZ);
        rx_bandwidth_hz = (uint32_t) ((tmp_u64) / ((uint64_t) S2LP_RX_BANDWIDTH_TABLE_FREQUENCY_KHZ));
    }
    return rx_bandwidth_hz;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
static S2LP_status_t _S2LP_compute_mantissa_exponent_rx_bandwidth(uint32_t rx_bandwidth_hz, S2LP_mantissa_exponent_t* rx_bandwidth_setting) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t table_index = 0;
    uint32_t rx_bandwidth = 0;
    // Check range.
    if (rx_bandwidth_hz < S2LP_RX_BANDWIDTH_HZ_MIN) {
        status = S2LP_ERROR_RX_BANDWIDTH_UNDERFLOW;
        goto errors;
    }
    if (rx_bandwidth_hz > S2LP_RX_BANDWIDTH_HZ_MAX) {
        status = S2LP_ERROR_RX_BANDWIDTH_OVERFLOW;
        goto errors;
    }
    // Index loop.
    for (table_index = 0; table_index < S2LP_RX_BANDWIDTH_TABLE_SIZE; table_index++) {
        // Compute RX bandwidth.
        rx_bandwidth = _S2LP_compute_rx_bandwidth(table_index);
        // Check result.
        if (rx_bandwidth <= rx_bandwidth_hz) break;
    }
    (*rx_bandwidth_setting).mantissa = (table_index % 9);
    (*rx_bandwidth_setting).exponent = (table_index / 9);
errors:
    return status;
}
#endif

/*** S2LP functions ***/

/*******************************************************************/
S2LP_status_t S2LP_init(void) {
    // Local variables.
    S2LP_status_t status;
    // Init hardware interface.
    status = S2LP_HW_init();
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_de_init(void) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    // Release hardware interface.
    status = S2LP_HW_de_init();
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_shutdown(uint8_t shutdown_enable) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    // Set GPIO.
    if (shutdown_enable == 0) {
        // Put SDN low.
        status = S2LP_HW_set_sdn_gpio(0);
        if (status != S2LP_SUCCESS) goto errors;
        // Wait for reset time.
        status = S2LP_HW_delay_milliseconds(S2LP_EXIT_SHUTDOWN_DELAY_MS);
        if (status != S2LP_SUCCESS) goto errors;
    }
    else {
        // Put SDN in high impedance (pull-up resistor used).
        status = S2LP_HW_set_sdn_gpio(1);
        if (status != S2LP_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_send_command(S2LP_command_t command) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t tx_data[S2LP_COMMAND_SPI_TRANSFER_SIZE];
    uint8_t rx_data[S2LP_COMMAND_SPI_TRANSFER_SIZE];
    // Check command.
    if (command >= S2LP_COMMAND_LAST) {
        status = S2LP_ERROR_COMMAND;
        goto errors;
    }
    // Write sequence.
    tx_data[0] = S2LP_HEADER_BYTE_COMMAND;
    tx_data[1] = command;
    status = S2LP_HW_spi_write_read_8(tx_data, rx_data, S2LP_COMMAND_SPI_TRANSFER_SIZE);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_wait_for_state(S2LP_state_t new_state) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t state = 0;
    uint8_t reg_value = 0;
    uint32_t loop_count = 0;
    // Poll MC_STATE until state is reached.
    do {
        status = _S2LP_read_register(S2LP_REGISTER_MC_STATE0, &reg_value);
        if (status != S2LP_SUCCESS) goto errors;
        state = (reg_value >> 1) & 0x7F;
        // Manage timeout.
        loop_count++;
        if (loop_count > S2LP_TIMEOUT_COUNT) {
            status = S2LP_ERROR_STATE_TIMEOUT;
            goto errors;
        }
    }
    while (state != new_state);
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_common_configuration(void) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    // Set PLL_CP_ISEL to '010'.
    status = _S2LP_read_register(S2LP_REGISTER_SYNT3, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value &= 0x1F;
    reg_value |= (0b010 << 5);
    status = _S2LP_write_register(S2LP_REGISTER_SYNT3, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set PLL_PFD_SPLIT_EN bit according to crystal frequency.
    status = _S2LP_read_register(S2LP_REGISTER_SYNTH_CONFIG2, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value &= 0xFB;
    reg_value |= (S2LP_PLL_PFD_SPLIT_EN_BIT << 2);
    status = _S2LP_write_register(S2LP_REGISTER_SYNTH_CONFIG2, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Disable equalizer, CS blanking and antenna switch.
    status = _S2LP_read_register(S2LP_REGISTER_ANT_SELECT_CONF, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value &= 0x83;
    status = _S2LP_write_register(S2LP_REGISTER_ANT_SELECT_CONF, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set clock recovery settings.
    status = _S2LP_write_register(S2LP_REGISTER_CLOCKREC2, 0x20);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_CLOCKREC1, 0x70);
    if (status != S2LP_SUCCESS) goto errors;
    // Disable PA power ramping and select slot 0.
    status = _S2LP_write_register(S2LP_REGISTER_PA_POWER0, 0x00);
    if (status != S2LP_SUCCESS) goto errors;
    // Disable FIR.
    status = _S2LP_read_register(S2LP_REGISTER_PA_CONFIG1, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value &= 0xFD;
    status = _S2LP_write_register(S2LP_REGISTER_PA_CONFIG1, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Enable interpolator.
    _S2LP_read_register(S2LP_REGISTER_MOD1, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value |= 0x80;
    status = _S2LP_write_register(S2LP_REGISTER_MOD1, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_oscillator(S2LP_oscillator_t oscillator) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameter.
    if (oscillator >= S2LP_OSCILLATOR_LAST) {
        status = S2LP_ERROR_OSCILLATOR;
        goto errors;
    }
    // Set RFDIV to 0, disable external RCO, configure EXT_REF bit.
    reg_value = (oscillator == S2LP_OSCILLATOR_TCXO) ? 0xB0 : 0x30;
    status = _S2LP_write_register(S2LP_REGISTER_XO_RCO_CONF0, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set digital clock divider according to crystal frequency.
    status = _S2LP_read_register(S2LP_REGISTER_XO_RCO_CONF1, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value &= 0xEF;
    reg_value |= (S2LP_PD_CLKDIB_BIT << 5);
    status = _S2LP_write_register(S2LP_REGISTER_XO_RCO_CONF1, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_wait_for_oscillator(void) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t xo_on = 0;
    uint8_t reg_value = 0;
    uint32_t loop_count = 0;
    // Poll MC_STATE until XO bit is set.
    do {
        status = _S2LP_read_register(S2LP_REGISTER_MC_STATE0, &reg_value);
        if (status != S2LP_SUCCESS) goto errors;
        xo_on = (reg_value & 0x01);
        // Manage timeout.
        loop_count++;
        if (loop_count > S2LP_TIMEOUT_COUNT) {
            status = S2LP_ERROR_OSCILLATOR_TIMEOUT;
            goto errors;
        }
    }
    while (xo_on == 0);
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_smps_frequency(uint32_t frequency_hz) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint64_t krm = 0;
    // Compute KRM value.
    krm = (((uint64_t) frequency_hz) << 15) / ((uint64_t) S2LP_FDIG_HZ);
    // Check value.
    if (krm > 0x7FFF) {
        status = S2LP_ERROR_SMPS_FREQUENCY_OVERFLOW;
        goto errors;
    }
    // Program registers.
    status = _S2LP_write_register(S2LP_REGISTER_PM_CONF3, (uint8_t) (((krm >> 8) & 0x7F) | 0x80));
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_PM_CONF2, (uint8_t) (krm & 0xFF));
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_modulation(S2LP_modulation_t modulation) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t mod2_reg_value = 0;
    // Check parameter.
    if (modulation >= S2LP_MODULATION_LAST) {
        status = S2LP_ERROR_MODULATION;
        goto errors;
    }
    // Read register.
    status = _S2LP_read_register(S2LP_REGISTER_MOD2, &mod2_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Change required bits.
    mod2_reg_value &= 0x0F;
    mod2_reg_value |= (modulation << 4);
    // Write register.
    status = _S2LP_write_register(S2LP_REGISTER_MOD2, mod2_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_rf_frequency(uint32_t frequency_hz) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint64_t synt_value = 0;
    uint8_t synt_reg_value = 0;
    // Check range.
    if (frequency_hz < S2LP_RF_FREQUENCY_HZ_MIN) {
        status = S2LP_ERROR_RF_FREQUENCY_UNDERFLOW;
        goto errors;
    }
    if (frequency_hz > S2LP_RF_FREQUENCY_HZ_MAX) {
        status = S2LP_ERROR_RF_FREQUENCY_OVERFLOW;
        goto errors;
    }
    // Set IF to 300kHz.
    status = _S2LP_write_register(S2LP_REGISTER_IF_OFFSET_ANA, S2LP_IF_OFFSET_ANA);
    if (status != S2LP_SUCCESS) goto errors;
    // See equation p.27 of S2LP datasheet.
    // Set CHNUM to 0.
    status = _S2LP_write_register(S2LP_REGISTER_CHNUM, 0x00);
    if (status != S2LP_SUCCESS) goto errors;
    // B=4 for 868MHz (high band, BS=0). REFDIV was set to 0 in oscillator configuration function.
    // SYNT = (fRF * 2^20 * B/2 * D) / (fXO) = (fRF * 2^21) / (fXO).
    synt_value = (((uint64_t) frequency_hz) << 21) / ((uint64_t) S2LP_DRIVER_XO_FREQUENCY_HZ);
    // Write registers.
    status = _S2LP_read_register(S2LP_REGISTER_SYNT3, &synt_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    synt_reg_value &= 0xE0; // BS=0 to select high band.
    synt_reg_value |= ((synt_value >> 24) & 0x0F);
    status = _S2LP_write_register(S2LP_REGISTER_SYNT3, synt_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    synt_reg_value = (synt_value >> 16) & 0xFF;
    status = _S2LP_write_register(S2LP_REGISTER_SYNT2, synt_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    synt_reg_value = (synt_value >> 8) & 0xFF;
    status = _S2LP_write_register(S2LP_REGISTER_SYNT1, synt_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    synt_reg_value = (synt_value >> 0) & 0xFF;
    status = _S2LP_write_register(S2LP_REGISTER_SYNT0, synt_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_fsk_deviation(uint32_t deviation_hz) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    S2LP_mantissa_exponent_t deviation_setting = { 0, 0 };
    uint8_t mod1_reg_value = 0;
    // Compute registers.
    status = _S2LP_compute_mantissa_exponent_deviation(deviation_hz, &deviation_setting);
    if (status != S2LP_SUCCESS) goto errors;
    // Write registers.
    status = _S2LP_write_register(S2LP_REGISTER_MOD0, deviation_setting.mantissa);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_read_register(S2LP_REGISTER_MOD1, &mod1_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    mod1_reg_value &= 0xF0;
    mod1_reg_value |= deviation_setting.exponent;
    status = _S2LP_write_register(S2LP_REGISTER_MOD1, mod1_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_datarate(uint32_t datarate_bps) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    S2LP_mantissa_exponent_t datarate_setting = { 0, 0 };
    uint8_t mod2_reg_value = 0;
    // Compute registers.
    status = _S2LP_compute_mantissa_exponent_datarate(datarate_bps, &datarate_setting);
    if (status != S2LP_SUCCESS) goto errors;
    // Write registers.
    status = _S2LP_write_register(S2LP_REGISTER_MOD4, (datarate_setting.mantissa >> 8) & 0x00FF);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_MOD3, (datarate_setting.mantissa >> 0) & 0x00FF);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_read_register(S2LP_REGISTER_MOD2, &mod2_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    mod2_reg_value &= 0xF0;
    mod2_reg_value |= (datarate_setting.exponent);
    status = _S2LP_write_register(S2LP_REGISTER_MOD2, mod2_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_packet_format(uint8_t packet_size_bytes, S2LP_crc_mode_t crc_mode) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameter.
    if (crc_mode >= S2LP_CRC_MODE_LAST) {
        status = S2LP_ERROR_CRC_MODE;
        goto errors;
    }
    // Set length.
    status = _S2LP_write_register(S2LP_REGISTER_PCKTLEN1, 0x00);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_PCKTLEN0, packet_size_bytes);
    if (status != S2LP_SUCCESS) goto errors;
    // Set CRC.
    status = _S2LP_read_register(S2LP_REGISTER_PCKTCTRL1, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set bits.
    reg_value &= 0x1F;
    reg_value |= (crc_mode << 5);
    // Write register.
    status = _S2LP_write_register(S2LP_REGISTER_PCKTCTRL1, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_configure_gpio(S2LP_gpio_t gpio, S2LP_gpio_mode_t mode, uint8_t function, S2LP_fifo_flag_direction_t fifo_flag_direction) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameters.
    if (gpio >= S2LP_GPIO_LAST) {
        status = S2LP_ERROR_GPIO_INDEX;
        goto errors;
    }
    if (mode >= S2LP_GPIO_MODE_LAST) {
        status = S2LP_ERROR_GPIO_MODE;
        goto errors;
    }
    if ((function >= S2LP_GPIO_INPUT_FUNCTION_LAST) && (function >= S2LP_GPIO_OUTPUT_FUNCTION_LAST)) {
        status = S2LP_ERROR_GPIO_FUNCTION;
        goto errors;
    }
    if (fifo_flag_direction >= S2LP_FIFO_FLAG_DIRECTION_LAST) {
        status = S2LP_ERROR_FIFO_FLAG_DIRECTION;
        goto errors;
    }
    // Read corresponding register.
    status = _S2LP_read_register((S2LP_REGISTER_GPIO0_CONF + gpio), &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set required bits.
    reg_value &= 0x04; // Bit 2 is reserved.
    reg_value |= ((mode & 0x02) << 0);
    reg_value |= ((function & 0x1F) << 3);
    // Write register.
    status = _S2LP_write_register((S2LP_REGISTER_GPIO0_CONF + gpio), reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Select FIFO flags.
    status = _S2LP_read_register(S2LP_REGISTER_PROTOCOL2, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value &= 0xFB;
    reg_value |= ((fifo_flag_direction & 0x01) << 2);
    status = _S2LP_write_register(S2LP_REGISTER_PROTOCOL2, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_configure_irq(S2LP_irq_index_t irq_index, uint8_t irq_enable) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    uint8_t reg_addr_offset = 0;
    uint8_t irq_bit_offset = 0;
    // Check parameter.
    if (irq_index >= S2LP_IRQ_INDEX_LAST) {
        status = S2LP_ERROR_IRQ_INDEX;
        goto errors;
    }
    // Get register and bit offsets.
    reg_addr_offset = (irq_index >> 3);
    irq_bit_offset = (irq_index % 8);
    // Read register.
    status = _S2LP_read_register((S2LP_REGISTER_IRQ_MASK0 - reg_addr_offset), &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set bit.
    reg_value &= ~(0b1 << irq_bit_offset);
    reg_value |= (((irq_enable == 0) ? 0b0 : 0b1) << irq_bit_offset);
    // Program register.
    status = _S2LP_write_register((S2LP_REGISTER_IRQ_MASK0 - reg_addr_offset), reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_get_irq_flag(S2LP_irq_index_t irq_index, uint8_t* irq_flag) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    uint8_t reg_addr_offset = 0;
    uint8_t irq_bit_offset = 0;
    // Check parameters.
    if (irq_index >= S2LP_IRQ_INDEX_LAST) {
        status = S2LP_ERROR_IRQ_INDEX;
        goto errors;
    }
    if (irq_flag == NULL) {
        status = S2LP_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Get register and bit offsets.
    reg_addr_offset = (irq_index >> 3);
    irq_bit_offset = (irq_index % 8);
    // Read register.
    status = _S2LP_read_register((S2LP_REGISTER_IRQ_STATUS0 - reg_addr_offset), &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Read bit.
    (*irq_flag) = (reg_value >> irq_bit_offset) & 0x01;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_disable_all_irq(void) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    // Reset all masks.
    status = _S2LP_write_register(S2LP_REGISTER_IRQ_MASK3, 0x00);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_IRQ_MASK2, 0x00);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_IRQ_MASK1, 0x00);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_IRQ_MASK0, 0x00);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_clear_all_irq(void) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    // Read IRQ status to clear flags.
    status = _S2LP_read_register(S2LP_REGISTER_IRQ_STATUS3, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_read_register(S2LP_REGISTER_IRQ_STATUS2, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_read_register(S2LP_REGISTER_IRQ_STATUS1, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_read_register(S2LP_REGISTER_IRQ_STATUS0, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
S2LP_status_t S2LP_set_fifo_threshold(S2LP_fifo_threshold_t fifo_threshold, uint8_t threshold_value) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    // Check parameters.
    if (fifo_threshold >= S2LP_FIFO_THRESHOLD_LAST) {
        status = S2LP_ERROR_FIFO_THRESHOLD;
        goto errors;
    }
    if (threshold_value > S2LP_FIFO_THHRESHOLD_BYTES_MAX) {
        status = S2LP_ERROR_FIFO_THRESHOLD_VALUE;
        goto errors;
    }
    // Write register.
    status = _S2LP_write_register(fifo_threshold, threshold_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}

#ifdef S2LP_DRIVER_TX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_set_rf_output_power(int8_t output_power_dbm) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    uint8_t pa_reg_value = 0;
    // Check parameter.
    if (output_power_dbm > S2LP_RF_OUTPUT_POWER_MAX) {
        status = S2LP_ERROR_RF_OUTPUT_POWER_OVERFLOW;
        goto errors;
    }
    if (output_power_dbm < S2LP_RF_OUTPUT_POWER_MIN) {
        status = S2LP_ERROR_RF_OUTPUT_POWER_UNDERFLOW;
        goto errors;
    }
    // Compute register value.
    pa_reg_value = (uint8_t) (29 - 2 * output_power_dbm);
    // Program register.
    status = _S2LP_read_register(S2LP_REGISTER_PA_POWER1, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    reg_value &= 0x80;
    reg_value |= (pa_reg_value & 0x7F);
    status = _S2LP_write_register(S2LP_REGISTER_PA_POWER1, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_TX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_set_tx_source(S2LP_tx_source_t tx_source) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameter.
    if (tx_source >= S2LP_TX_SOURCE_LAST) {
        status = S2LP_ERROR_TX_SOURCE;
        goto errors;
    }
    // Read register.
    status = _S2LP_read_register(S2LP_REGISTER_PCKTCTRL1, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set bits.
    reg_value &= 0xF3;
    reg_value |= (tx_source << 2);
    // Write register.
    status = _S2LP_write_register(S2LP_REGISTER_PCKTCTRL1, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_TX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_write_fifo(uint8_t* fifo_data, uint8_t fifo_data_size) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t tx_data[S2LP_FIFO_SPI_TRANSFER_SIZE + S2LP_FIFO_SIZE_BYTES];
    uint8_t rx_data[S2LP_FIFO_SPI_TRANSFER_SIZE + S2LP_FIFO_SIZE_BYTES];
    uint8_t idx = 0;
    // Check parameters.
    if (fifo_data == NULL) {
        status = S2LP_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (fifo_data_size > S2LP_FIFO_SIZE_BYTES) {
        status = S2LP_ERROR_TX_FIFO_DATA_SIZE;
        goto errors;
    }
    // Build FIFO header.
    tx_data[0] = S2LP_HEADER_BYTE_WRITE;
    tx_data[1] = S2LP_REGISTER_FIFO;
    for (idx = 0; idx < fifo_data_size; idx++) {
        tx_data[S2LP_FIFO_SPI_TRANSFER_SIZE + idx] = fifo_data[idx];
    }
    // Write sequence.
    status = S2LP_HW_spi_write_read_8(tx_data, rx_data, (S2LP_FIFO_SPI_TRANSFER_SIZE + fifo_data_size));
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_set_rx_bandwidth(uint32_t rx_bandwidth_hz, S2LP_afc_mode_t afc_mode) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    S2LP_mantissa_exponent_t rx_bandwidth_setting = { 0, 0 };
    uint8_t reg_value = 0;
    // Compute registers.
    status = _S2LP_compute_mantissa_exponent_rx_bandwidth(rx_bandwidth_hz, &rx_bandwidth_setting);
    if (status != S2LP_SUCCESS) goto errors;
    // Write register.
    reg_value = ((rx_bandwidth_setting.mantissa << 4) & 0xF0) + (rx_bandwidth_setting.exponent & 0x0F);
    status = _S2LP_write_register(S2LP_REGISTER_CHFLT, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Read register.
    status = _S2LP_read_register(S2LP_REGISTER_AFC2, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Reset bits.
    reg_value &= 0x1F;
    // Set AFC mode.
    switch (afc_mode) {
    case S2LP_AFC_MODE_DISABLE:
        // Nothing to do.
        break;
    case S2LP_AFC_MODE_CONTINUOUS:
        reg_value |= 0x40;
        break;
    case S2LP_AFC_MODE_FREEZE:
        reg_value |= 0xC0;
        break;
    default:
        status = S2LP_ERROR_AFC_MODE;
        goto errors;
    }
    // Program register.
    status = _S2LP_write_register(S2LP_REGISTER_AFC2, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_set_rssi_threshold(int16_t rssi_threshold_dbm) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    // Check parameter.
    if ((rssi_threshold_dbm < (0 - S2LP_RSSI_OFFSET_DB)) || (rssi_threshold_dbm > (255 - S2LP_RSSI_OFFSET_DB))) {
        status = S2LP_ERROR_RSSI_THRESHOLD;
        goto errors;
    }
    // Program register.
    status = _S2LP_write_register(S2LP_REGISTER_RSSI_TH, (uint8_t) (rssi_threshold_dbm + S2LP_RSSI_OFFSET_DB));
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_get_rssi(S2LP_rssi_t rssi_type, int16_t* rssi_dbm) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t rssi_level_reg_value = 0;
    // Check parameter.
    if (rssi_dbm == NULL) {
        status = S2LP_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read accurate register.
    switch (rssi_type) {
    case S2LP_RSSI_TYPE_RUN:
        status = _S2LP_read_register(S2LP_REGISTER_RSSI_LEVEL_RUN, &rssi_level_reg_value);
        break;
    case S2LP_RSSI_TYPE_SYNC_WORD:
        status = _S2LP_read_register(S2LP_REGISTER_RSSI_LEVEL, &rssi_level_reg_value);
        break;
    default:
        status = S2LP_ERROR_RSSI_TYPE;
        break;
    }
    if (status != S2LP_SUCCESS) goto errors;
    // Convert to dBm.
    (*rssi_dbm) = (((int16_t) rssi_level_reg_value) - ((int16_t) S2LP_RSSI_OFFSET_DB));
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_set_preamble_detector(uint8_t preamble_length_2bits, S2LP_preamble_pattern_t preamble_pattern) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t pcktctrlx_reg_value = 0;
    // Check parameter.
    if (preamble_pattern >= S2LP_PREAMBLE_PATTERN_LAST) {
        status = S2LP_ERROR_PREAMBLE_PATTERN;
        goto errors;
    }
    // Set length.
    status = _S2LP_read_register(S2LP_REGISTER_PCKTCTRL6, &pcktctrlx_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    pcktctrlx_reg_value &= 0xFC;
    status = _S2LP_write_register(S2LP_REGISTER_PCKTCTRL6, pcktctrlx_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    status = _S2LP_write_register(S2LP_REGISTER_PCKTCTRL5, preamble_length_2bits);
    if (status != S2LP_SUCCESS) goto errors;
    // Set pattern.
    status = _S2LP_read_register(S2LP_REGISTER_PCKTCTRL3, &pcktctrlx_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    pcktctrlx_reg_value &= 0xFC;
    pcktctrlx_reg_value |= (preamble_pattern & 0x03);
    status = _S2LP_write_register(S2LP_REGISTER_PCKTCTRL3, pcktctrlx_reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_set_sync_word(uint8_t* sync_word, uint8_t sync_word_length_bits) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t sync_word_length_bytes = 0;
    uint8_t generic_byte = 0;
    // Check parameters.
    if (sync_word == NULL) {
        status = S2LP_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (sync_word_length_bits > S2LP_SYNC_WORD_LENGTH_BITS_MAX) {
        status = S2LP_ERROR_SYNC_WORD_SIZE;
        goto errors;
    }
    // Set synchronization word.
    sync_word_length_bytes = (sync_word_length_bits / 8);
    if ((sync_word_length_bits - (sync_word_length_bytes * 8)) > 0) {
        sync_word_length_bytes++;
    }
    for (generic_byte = 0; generic_byte < sync_word_length_bytes; generic_byte++) {
        status = _S2LP_write_register((S2LP_REGISTER_SYNC0 - generic_byte), sync_word[generic_byte]);
        if (status != S2LP_SUCCESS) goto errors;
    }
    // Set length.
    status = _S2LP_read_register(S2LP_REGISTER_PCKTCTRL6, &generic_byte);
    if (status != S2LP_SUCCESS) goto errors;
    generic_byte &= 0x03;
    generic_byte |= (sync_word_length_bits << 2);
    status = _S2LP_write_register(S2LP_REGISTER_PCKTCTRL6, generic_byte);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_set_rx_source(S2LP_rx_source_t rx_source) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameter.
    if (rx_source >= S2LP_RX_SOURCE_LAST) {
        status = S2LP_ERROR_RX_SOURCE;
        goto errors;
    }
    // Read register.
    status = _S2LP_read_register(S2LP_REGISTER_PCKTCTRL3, &reg_value);
    if (status != S2LP_SUCCESS) goto errors;
    // Set bits.
    reg_value &= 0xCF;
    reg_value |= (rx_source << 4);
    // Write register.
    status = _S2LP_write_register(S2LP_REGISTER_PCKTCTRL3, reg_value);
    if (status != S2LP_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef S2LP_DRIVER_RX_ENABLE
/*******************************************************************/
S2LP_status_t S2LP_read_fifo(uint8_t* fifo_data, uint8_t fifo_data_size) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    uint8_t tx_data[S2LP_FIFO_SPI_TRANSFER_SIZE + S2LP_FIFO_SIZE_BYTES];
    uint8_t rx_data[S2LP_FIFO_SPI_TRANSFER_SIZE + S2LP_FIFO_SIZE_BYTES];
    uint8_t idx = 0;
    // Check parameters.
    if (fifo_data == NULL) {
        status = S2LP_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (fifo_data_size > S2LP_FIFO_SIZE_BYTES) {
        status = S2LP_ERROR_RX_FIFO_DATA_SIZE;
        goto errors;
    }
    // Build FIFO header.
    tx_data[0] = S2LP_HEADER_BYTE_READ;
    tx_data[1] = S2LP_REGISTER_FIFO;
    for (idx = 0; idx < fifo_data_size; idx++) {
        tx_data[S2LP_FIFO_SPI_TRANSFER_SIZE + idx] = 0;
    }
    // Read sequence.
    status = S2LP_HW_spi_write_read_8(tx_data, rx_data, (S2LP_FIFO_SPI_TRANSFER_SIZE + fifo_data_size));
    if (status != S2LP_SUCCESS) goto errors;
    // Extract FIFO data.
    for (idx = 0; idx < fifo_data_size; idx++) {
        fifo_data[idx] = rx_data[S2LP_FIFO_SPI_TRANSFER_SIZE + idx];
    }
errors:
    return status;
}
#endif

#endif /* S2LP_DRIVER_DISABLE */
