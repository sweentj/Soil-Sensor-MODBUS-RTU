#ifndef SOIL_MODBUS_RTU_H
#define SOIL_MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// Sensor register map from CWT-Soil-THC-S manual (Modbus RTU)
#define SOIL_THC_REG_HUMIDITY           0x0000U
#define SOIL_THC_REG_TEMPERATURE        0x0001U
#define SOIL_THC_REG_CONDUCTIVITY       0x0002U
#define SOIL_THC_REG_SALINITY           0x0003U
#define SOIL_THC_REG_TDS                0x0004U
#define SOIL_THC_REG_COND_FACTOR        0x0022U
#define SOIL_THC_REG_SAL_FACTOR         0x0023U
#define SOIL_THC_REG_TDS_FACTOR         0x0024U
#define SOIL_THC_REG_TEMP_OFFSET        0x0050U
#define SOIL_THC_REG_HUMIDITY_OFFSET    0x0051U
#define SOIL_THC_REG_CONDUCTIVITY_OFFSET 0x0052U
#define SOIL_THC_REG_SLAVE_ID           0x07D0U
#define SOIL_THC_REG_BAUD               0x07D1U

#define SOIL_THC_BAUD_2400_CODE         0U
#define SOIL_THC_BAUD_4800_CODE         1U
#define SOIL_THC_BAUD_9600_CODE         2U

#define SOIL_MODBUS_BROADCAST_ADDRESS   0xFFU

typedef enum soil_modbus_status_t {
    SOIL_MODBUS_STATUS_OK = 0,
    SOIL_MODBUS_STATUS_BAD_ARG,
    SOIL_MODBUS_STATUS_UART,
    SOIL_MODBUS_STATUS_TIMEOUT,
    SOIL_MODBUS_STATUS_CRC,
    SOIL_MODBUS_STATUS_PROTOCOL,
    SOIL_MODBUS_STATUS_EXCEPTION
} soil_modbus_status_t;

typedef struct soil_modbus_rtu_config_t {
    void *uart_handle;
    uint32_t de_pin;
    bool has_de_pin;
    bool de_active_high;
    bool use_interrupt_read;
    uint8_t uart_instance;
    uint32_t tx_timeout_ms;
    uint32_t first_byte_timeout_ms;
    uint32_t inter_byte_timeout_ms;
    uint32_t inter_frame_delay_us;
    uint32_t turnaround_delay_us;
} soil_modbus_rtu_config_t;

typedef struct soil_modbus_rtu_ctx_t {
    soil_modbus_rtu_config_t cfg;
    uint8_t last_exception;
} soil_modbus_rtu_ctx_t;

typedef struct soil_thc_triplet_t {
    uint16_t humidity_deci_percent;
    int16_t temperature_deci_c;
    uint16_t conductivity_us_cm;
} soil_thc_triplet_t;

typedef struct soil_thc_reading_t {
    uint16_t humidity_deci_percent;
    int16_t temperature_deci_c;
    uint16_t conductivity_us_cm;
    uint16_t salinity_raw;
    uint16_t tds_raw;
} soil_thc_reading_t;

void soil_modbus_rtu_config_init_default(soil_modbus_rtu_config_t *cfg, void *uart_handle, uint32_t de_pin);
soil_modbus_status_t soil_modbus_rtu_init(soil_modbus_rtu_ctx_t *ctx, const soil_modbus_rtu_config_t *cfg);

soil_modbus_status_t soil_modbus_rtu_read_holding_registers(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id,
                                                            uint16_t start_address, uint16_t quantity,
                                                            uint16_t *registers_out);

soil_modbus_status_t soil_modbus_rtu_write_single_register(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id,
                                                           uint16_t address, uint16_t value);

soil_modbus_status_t soil_thc_read_triplet(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id, soil_thc_triplet_t *out);
soil_modbus_status_t soil_thc_read_all(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id, soil_thc_reading_t *out);
soil_modbus_status_t soil_thc_set_slave_id(soil_modbus_rtu_ctx_t *ctx, uint8_t current_slave_id, uint8_t new_slave_id);
soil_modbus_status_t soil_thc_set_baud_code(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id, uint16_t baud_code);
soil_modbus_status_t soil_thc_enquire_slave_id(soil_modbus_rtu_ctx_t *ctx, uint8_t *slave_id_out);

uint8_t soil_modbus_rtu_last_exception(const soil_modbus_rtu_ctx_t *ctx);
const char *soil_modbus_status_to_string(soil_modbus_status_t status);

#ifdef __cplusplus
}
#endif

#endif
