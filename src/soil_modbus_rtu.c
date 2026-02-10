#include "soil_modbus_rtu.h"

#include "am_bsp.h"
#include "am_mcu_apollo.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

#define SOIL_MODBUS_FC_READ_HOLDING_REGISTERS 0x03U
#define SOIL_MODBUS_FC_WRITE_SINGLE_REGISTER   0x06U

#define SOIL_MODBUS_MAX_REGISTERS_PER_REQ      123U
#define SOIL_MODBUS_MAX_RTU_FRAME              256U
#define SOIL_MODBUS_IRQ_RX_BUFFER_SIZE         256U

static volatile uint16_t g_irq_rx_head = 0U;
static volatile uint16_t g_irq_rx_tail = 0U;
static volatile uint32_t g_irq_rx_overflow = 0U;
static uint8_t g_irq_rx_buffer[SOIL_MODBUS_IRQ_RX_BUFFER_SIZE];
static SemaphoreHandle_t g_irq_rx_sem = NULL;
static soil_modbus_rtu_ctx_t *g_irq_rx_ctx = NULL;

static void irq_rx_reset(void)
{
    taskENTER_CRITICAL();
    g_irq_rx_head = 0U;
    g_irq_rx_tail = 0U;
    g_irq_rx_overflow = 0U;
    taskEXIT_CRITICAL();
}

static uint16_t irq_rx_pop_bytes(uint8_t *dst, uint16_t max_len)
{
    uint16_t copied = 0U;

    taskENTER_CRITICAL();
    while ((copied < max_len) && (g_irq_rx_tail != g_irq_rx_head))
    {
        dst[copied++] = g_irq_rx_buffer[g_irq_rx_tail];
        g_irq_rx_tail++;
        if (g_irq_rx_tail >= SOIL_MODBUS_IRQ_RX_BUFFER_SIZE)
        {
            g_irq_rx_tail = 0U;
        }
    }
    taskEXIT_CRITICAL();

    return copied;
}

static void irq_rx_push_byte(uint8_t byte)
{
    uint16_t next = g_irq_rx_head + 1U;

    if (next >= SOIL_MODBUS_IRQ_RX_BUFFER_SIZE)
    {
        next = 0U;
    }

    if (next == g_irq_rx_tail)
    {
        g_irq_rx_overflow++;
        return;
    }

    g_irq_rx_buffer[g_irq_rx_head] = byte;
    g_irq_rx_head = next;
}

static void soil_modbus_uart_irq_service(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t fifo_buf[16];
    uint32_t irq_status = 0U;
    uint32_t bytes = 0U;
    uint32_t status;
    bool got_data = false;

    if ((g_irq_rx_ctx == NULL) || !g_irq_rx_ctx->cfg.use_interrupt_read)
    {
        return;
    }

    status = am_hal_uart_interrupt_status_get(g_irq_rx_ctx->cfg.uart_handle, &irq_status, true);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return;
    }

    (void)am_hal_uart_interrupt_clear(g_irq_rx_ctx->cfg.uart_handle, irq_status);

    do
    {
        bytes = 0U;
        status = am_hal_uart_fifo_read(g_irq_rx_ctx->cfg.uart_handle, fifo_buf, sizeof(fifo_buf), &bytes);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            break;
        }

        if (bytes != 0U)
        {
            uint32_t i;
            got_data = true;
            for (i = 0U; i < bytes; i++)
            {
                irq_rx_push_byte(fifo_buf[i]);
            }
        }
    } while (bytes != 0U);

    if (got_data && (g_irq_rx_sem != NULL))
    {
        (void)xSemaphoreGiveFromISR(g_irq_rx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void am_uart1_isr(void)
{
    soil_modbus_uart_irq_service();
}

static uint16_t soil_modbus_crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        uint8_t j;
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint16_t be16_to_u16(const uint8_t *in)
{
    return ((uint16_t)in[0] << 8) | (uint16_t)in[1];
}

static void u16_to_be16(uint16_t value, uint8_t *out)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)(value & 0xFFU);
}

static void delay_us(uint32_t us)
{
    if (us == 0U)
    {
        return;
    }

    am_hal_flash_delay(FLASH_CYCLES_US(us));
}

static void set_driver_enable(const soil_modbus_rtu_ctx_t *ctx, bool tx_mode)
{
    if ((ctx == NULL) || !ctx->cfg.has_de_pin)
    {
        return;
    }

    if (tx_mode == ctx->cfg.de_active_high)
    {
        (void)am_hal_gpio_state_write(ctx->cfg.de_pin, AM_HAL_GPIO_OUTPUT_SET);
    }
    else
    {
        (void)am_hal_gpio_state_write(ctx->cfg.de_pin, AM_HAL_GPIO_OUTPUT_CLEAR);
    }
}

static soil_modbus_status_t uart_read_exact_polling(soil_modbus_rtu_ctx_t *ctx, uint8_t *data, uint16_t len,
                                                    uint32_t timeout_ms)
{
    uint32_t total = 0U;
    uint32_t idle_us = 0U;
    const uint32_t timeout_us = timeout_ms * 1000U;
    const uint32_t poll_us = 100U;

    while (total < (uint32_t)len)
    {
        uint32_t bytes = 0U;
        uint32_t status = am_hal_uart_fifo_read(ctx->cfg.uart_handle, data + total, (uint32_t)len - total, &bytes);

        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return SOIL_MODBUS_STATUS_UART;
        }

        if (bytes != 0U)
        {
            total += bytes;
            idle_us = 0U;
            continue;
        }

        if ((timeout_us != 0U) && (idle_us >= timeout_us))
        {
            return SOIL_MODBUS_STATUS_TIMEOUT;
        }

        delay_us(poll_us);
        idle_us += poll_us;
    }

    return SOIL_MODBUS_STATUS_OK;
}

static soil_modbus_status_t uart_read_exact_interrupt(soil_modbus_rtu_ctx_t *ctx, uint8_t *data, uint16_t len,
                                                      uint32_t timeout_ms)
{
    uint16_t total = 0U;
    TickType_t idle_start;
    TickType_t timeout_ticks;

    if ((ctx == NULL) || (data == NULL) || (g_irq_rx_sem == NULL))
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if ((timeout_ms != 0U) && (timeout_ticks == 0U))
    {
        timeout_ticks = 1U;
    }
    idle_start = xTaskGetTickCount();

    while (total < len)
    {
        uint16_t copied = irq_rx_pop_bytes(data + total, (uint16_t)(len - total));
        if (copied != 0U)
        {
            total = (uint16_t)(total + copied);
            idle_start = xTaskGetTickCount();
            continue;
        }

        if (timeout_ms == 0U)
        {
            return SOIL_MODBUS_STATUS_TIMEOUT;
        }

        if ((xTaskGetTickCount() - idle_start) >= timeout_ticks)
        {
            return SOIL_MODBUS_STATUS_TIMEOUT;
        }

        if (xSemaphoreTake(g_irq_rx_sem, timeout_ticks) != pdPASS)
        {
            return SOIL_MODBUS_STATUS_TIMEOUT;
        }
    }

    return SOIL_MODBUS_STATUS_OK;
}

static soil_modbus_status_t uart_read_exact(soil_modbus_rtu_ctx_t *ctx, uint8_t *data, uint16_t len,
                                            uint32_t timeout_ms)
{
    if ((ctx != NULL) && ctx->cfg.use_interrupt_read)
    {
        return uart_read_exact_interrupt(ctx, data, len, timeout_ms);
    }

    return uart_read_exact_polling(ctx, data, len, timeout_ms);
}

static soil_modbus_status_t uart_write_exact(soil_modbus_rtu_ctx_t *ctx, const uint8_t *data, uint16_t len)
{
    am_hal_uart_transfer_t transfer;
    uint32_t bytes = 0;
    uint32_t status;

    memset(&transfer, 0, sizeof(transfer));
    transfer.ui32Direction = AM_HAL_UART_WRITE;
    transfer.pui8Data = (uint8_t *)data;
    transfer.ui32NumBytes = len;
    transfer.ui32TimeoutMs = ctx->cfg.tx_timeout_ms;
    transfer.pui32BytesTransferred = &bytes;

    status = am_hal_uart_transfer(ctx->cfg.uart_handle, &transfer);
    if ((status != AM_HAL_STATUS_SUCCESS) || (bytes != len))
    {
        return (status == AM_HAL_STATUS_TIMEOUT) ? SOIL_MODBUS_STATUS_TIMEOUT : SOIL_MODBUS_STATUS_UART;
    }

    status = am_hal_uart_tx_flush(ctx->cfg.uart_handle);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return SOIL_MODBUS_STATUS_UART;
    }

    return SOIL_MODBUS_STATUS_OK;
}

static void uart_flush_rx(soil_modbus_rtu_ctx_t *ctx)
{
    uint8_t trash[32];
    uint32_t bytes = 0U;
    uint32_t status = AM_HAL_STATUS_SUCCESS;

    do
    {
        bytes = 0U;
        status = am_hal_uart_fifo_read(ctx->cfg.uart_handle, trash, sizeof(trash), &bytes);
    } while ((status == AM_HAL_STATUS_SUCCESS) && (bytes != 0U));
}

static soil_modbus_status_t read_exception_tail_if_needed(soil_modbus_rtu_ctx_t *ctx, uint8_t response_fc,
                                                          uint8_t request_fc, uint8_t *hdr)
{
    if (response_fc == (uint8_t)(request_fc | 0x80U))
    {
        uint8_t tail[2];
        uint8_t frame[5];
        soil_modbus_status_t status;
        uint16_t received_crc;
        uint16_t computed_crc;

        status = uart_read_exact(ctx, tail, sizeof(tail), ctx->cfg.inter_byte_timeout_ms);
        if (status != SOIL_MODBUS_STATUS_OK)
        {
            return status;
        }

        frame[0] = hdr[0];
        frame[1] = hdr[1];
        frame[2] = hdr[2];
        frame[3] = tail[0];
        frame[4] = tail[1];

        received_crc = ((uint16_t)tail[1] << 8) | (uint16_t)tail[0];
        computed_crc = soil_modbus_crc16(frame, 3U);
        if (received_crc != computed_crc)
        {
            return SOIL_MODBUS_STATUS_CRC;
        }

        ctx->last_exception = hdr[2];
        return SOIL_MODBUS_STATUS_EXCEPTION;
    }

    return SOIL_MODBUS_STATUS_OK;
}

static soil_modbus_status_t tx_request_begin(soil_modbus_rtu_ctx_t *ctx)
{
    if ((ctx == NULL) || (ctx->cfg.uart_handle == NULL))
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    ctx->last_exception = 0U;
    delay_us(ctx->cfg.inter_frame_delay_us);
    if (ctx->cfg.use_interrupt_read)
    {
        irq_rx_reset();
    }
    uart_flush_rx(ctx);
    set_driver_enable(ctx, true);
    return SOIL_MODBUS_STATUS_OK;
}

static void tx_request_end(soil_modbus_rtu_ctx_t *ctx)
{
    set_driver_enable(ctx, false);
}

static soil_modbus_status_t irq_read_backend_enable(soil_modbus_rtu_ctx_t *ctx)
{
    uint32_t status;

    if ((ctx == NULL) || !ctx->cfg.use_interrupt_read)
    {
        return SOIL_MODBUS_STATUS_OK;
    }

    if (ctx->cfg.uart_instance != 1U)
    {
        // UART0 ISR is owned by console_task in this project.
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    if (g_irq_rx_sem == NULL)
    {
        g_irq_rx_sem = xSemaphoreCreateBinary();
        if (g_irq_rx_sem == NULL)
        {
            return SOIL_MODBUS_STATUS_UART;
        }
    }

    while (xSemaphoreTake(g_irq_rx_sem, 0U) == pdPASS)
    {
    }

    irq_rx_reset();
    g_irq_rx_ctx = ctx;

    status = am_hal_uart_interrupt_disable(ctx->cfg.uart_handle, AM_HAL_UART_INT_TX);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return SOIL_MODBUS_STATUS_UART;
    }

    status = am_hal_uart_interrupt_clear(ctx->cfg.uart_handle, 0xFFFFFFFFU);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return SOIL_MODBUS_STATUS_UART;
    }

    status = am_hal_uart_interrupt_enable(ctx->cfg.uart_handle,
                                          AM_HAL_UART_INT_RX |
                                          AM_HAL_UART_INT_RX_TMOUT |
                                          AM_HAL_UART_INT_OVER_RUN |
                                          AM_HAL_UART_INT_BREAK_ERR |
                                          AM_HAL_UART_INT_PARITY_ERR |
                                          AM_HAL_UART_INT_FRAME_ERR);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return SOIL_MODBUS_STATUS_UART;
    }

    NVIC_SetPriority(UART1_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_EnableIRQ(UART1_IRQn);

    return SOIL_MODBUS_STATUS_OK;
}

void soil_modbus_rtu_config_init_default(soil_modbus_rtu_config_t *cfg, void *uart_handle, uint32_t de_pin)
{
    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->uart_handle = uart_handle;
    cfg->de_pin = de_pin;
    cfg->has_de_pin = true;
    cfg->de_active_high = true;
    cfg->use_interrupt_read = false;
    cfg->uart_instance = 1U;
    cfg->tx_timeout_ms = 100U;
    cfg->first_byte_timeout_ms = 250U;
    cfg->inter_byte_timeout_ms = 60U;
    cfg->inter_frame_delay_us = 8000U;
    cfg->turnaround_delay_us = 200U;
}

soil_modbus_status_t soil_modbus_rtu_init(soil_modbus_rtu_ctx_t *ctx, const soil_modbus_rtu_config_t *cfg)
{
    soil_modbus_status_t status;

    if ((ctx == NULL) || (cfg == NULL) || (cfg->uart_handle == NULL))
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    set_driver_enable(ctx, false);
    status = irq_read_backend_enable(ctx);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    return SOIL_MODBUS_STATUS_OK;
}

soil_modbus_status_t soil_modbus_rtu_read_holding_registers(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id,
                                                            uint16_t start_address, uint16_t quantity,
                                                            uint16_t *registers_out)
{
    uint8_t request[8];
    uint8_t header[3];
    uint8_t payload[SOIL_MODBUS_MAX_RTU_FRAME];
    uint16_t i;
    uint16_t payload_len;
    uint16_t received_crc;
    uint16_t computed_crc;
    uint16_t expected_bytes;
    soil_modbus_status_t status;

    if ((ctx == NULL) || (registers_out == NULL) || (quantity == 0U) || (quantity > SOIL_MODBUS_MAX_REGISTERS_PER_REQ))
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    expected_bytes = (uint16_t)(quantity * 2U);
    if ((uint16_t)(expected_bytes + 5U) > SOIL_MODBUS_MAX_RTU_FRAME)
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    request[0] = slave_id;
    request[1] = SOIL_MODBUS_FC_READ_HOLDING_REGISTERS;
    u16_to_be16(start_address, &request[2]);
    u16_to_be16(quantity, &request[4]);
    computed_crc = soil_modbus_crc16(request, 6U);
    request[6] = (uint8_t)(computed_crc & 0xFFU);
    request[7] = (uint8_t)(computed_crc >> 8);

    status = tx_request_begin(ctx);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    status = uart_write_exact(ctx, request, sizeof(request));
    tx_request_end(ctx);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    delay_us(ctx->cfg.turnaround_delay_us);

    status = uart_read_exact(ctx, &header[0], 1U, ctx->cfg.first_byte_timeout_ms);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    status = uart_read_exact(ctx, &header[1], 2U, ctx->cfg.inter_byte_timeout_ms);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    if (header[0] != slave_id)
    {
        return SOIL_MODBUS_STATUS_PROTOCOL;
    }

    status = read_exception_tail_if_needed(ctx, header[1], SOIL_MODBUS_FC_READ_HOLDING_REGISTERS, header);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    if (header[1] != SOIL_MODBUS_FC_READ_HOLDING_REGISTERS)
    {
        return SOIL_MODBUS_STATUS_PROTOCOL;
    }

    if (header[2] != (uint8_t)expected_bytes)
    {
        return SOIL_MODBUS_STATUS_PROTOCOL;
    }

    payload_len = (uint16_t)header[2] + 2U;
    status = uart_read_exact(ctx, payload, payload_len, ctx->cfg.inter_byte_timeout_ms);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    received_crc = ((uint16_t)payload[payload_len - 1] << 8) | (uint16_t)payload[payload_len - 2];

    {
        uint8_t frame[SOIL_MODBUS_MAX_RTU_FRAME];
        frame[0] = header[0];
        frame[1] = header[1];
        frame[2] = header[2];
        memcpy(&frame[3], payload, payload_len - 2U);
        computed_crc = soil_modbus_crc16(frame, (uint16_t)(3U + payload_len - 2U));
    }

    if (received_crc != computed_crc)
    {
        return SOIL_MODBUS_STATUS_CRC;
    }

    for (i = 0U; i < quantity; i++)
    {
        registers_out[i] = be16_to_u16(&payload[i * 2U]);
    }

    return SOIL_MODBUS_STATUS_OK;
}

soil_modbus_status_t soil_modbus_rtu_write_single_register(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id,
                                                           uint16_t address, uint16_t value)
{
    uint8_t request[8];
    uint8_t response[8];
    soil_modbus_status_t status;
    uint16_t computed_crc;
    uint16_t received_crc;

    if (ctx == NULL)
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    request[0] = slave_id;
    request[1] = SOIL_MODBUS_FC_WRITE_SINGLE_REGISTER;
    u16_to_be16(address, &request[2]);
    u16_to_be16(value, &request[4]);
    computed_crc = soil_modbus_crc16(request, 6U);
    request[6] = (uint8_t)(computed_crc & 0xFFU);
    request[7] = (uint8_t)(computed_crc >> 8);

    status = tx_request_begin(ctx);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    status = uart_write_exact(ctx, request, sizeof(request));
    tx_request_end(ctx);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    delay_us(ctx->cfg.turnaround_delay_us);

    status = uart_read_exact(ctx, &response[0], 1U, ctx->cfg.first_byte_timeout_ms);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    status = uart_read_exact(ctx, &response[1], 2U, ctx->cfg.inter_byte_timeout_ms);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    if (response[1] == (uint8_t)(SOIL_MODBUS_FC_WRITE_SINGLE_REGISTER | 0x80U))
    {
        uint8_t tail[2];

        status = uart_read_exact(ctx, tail, sizeof(tail), ctx->cfg.inter_byte_timeout_ms);
        if (status != SOIL_MODBUS_STATUS_OK)
        {
            return status;
        }

        response[3] = tail[0];
        response[4] = tail[1];
        received_crc = ((uint16_t)tail[1] << 8) | (uint16_t)tail[0];
        computed_crc = soil_modbus_crc16(response, 3U);
        if (received_crc != computed_crc)
        {
            return SOIL_MODBUS_STATUS_CRC;
        }

        ctx->last_exception = response[2];
        return SOIL_MODBUS_STATUS_EXCEPTION;
    }

    status = uart_read_exact(ctx, &response[3], 5U, ctx->cfg.inter_byte_timeout_ms);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    if ((response[0] != slave_id) || (response[1] != SOIL_MODBUS_FC_WRITE_SINGLE_REGISTER))
    {
        return SOIL_MODBUS_STATUS_PROTOCOL;
    }

    if (be16_to_u16(&response[2]) != address)
    {
        return SOIL_MODBUS_STATUS_PROTOCOL;
    }

    if (be16_to_u16(&response[4]) != value)
    {
        return SOIL_MODBUS_STATUS_PROTOCOL;
    }

    received_crc = ((uint16_t)response[7] << 8) | (uint16_t)response[6];
    computed_crc = soil_modbus_crc16(response, 6U);
    if (received_crc != computed_crc)
    {
        return SOIL_MODBUS_STATUS_CRC;
    }

    return SOIL_MODBUS_STATUS_OK;
}

soil_modbus_status_t soil_thc_read_triplet(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id, soil_thc_triplet_t *out)
{
    uint16_t regs[3];
    soil_modbus_status_t status;

    if (out == NULL)
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    status = soil_modbus_rtu_read_holding_registers(ctx, slave_id, SOIL_THC_REG_HUMIDITY, 3U, regs);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    out->humidity_deci_percent = regs[0];
    out->temperature_deci_c = (int16_t)regs[1];
    out->conductivity_us_cm = regs[2];
    return SOIL_MODBUS_STATUS_OK;
}

soil_modbus_status_t soil_thc_read_all(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id, soil_thc_reading_t *out)
{
    uint16_t regs[5];
    soil_modbus_status_t status;

    if (out == NULL)
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    status = soil_modbus_rtu_read_holding_registers(ctx, slave_id, SOIL_THC_REG_HUMIDITY, 5U, regs);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    out->humidity_deci_percent = regs[0];
    out->temperature_deci_c = (int16_t)regs[1];
    out->conductivity_us_cm = regs[2];
    out->salinity_raw = regs[3];
    out->tds_raw = regs[4];
    return SOIL_MODBUS_STATUS_OK;
}

soil_modbus_status_t soil_thc_set_slave_id(soil_modbus_rtu_ctx_t *ctx, uint8_t current_slave_id, uint8_t new_slave_id)
{
    if ((new_slave_id == 0U) || (new_slave_id == SOIL_MODBUS_BROADCAST_ADDRESS))
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    return soil_modbus_rtu_write_single_register(ctx, current_slave_id, SOIL_THC_REG_SLAVE_ID, new_slave_id);
}

soil_modbus_status_t soil_thc_set_baud_code(soil_modbus_rtu_ctx_t *ctx, uint8_t slave_id, uint16_t baud_code)
{
    if (baud_code > SOIL_THC_BAUD_9600_CODE)
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    return soil_modbus_rtu_write_single_register(ctx, slave_id, SOIL_THC_REG_BAUD, baud_code);
}

soil_modbus_status_t soil_thc_enquire_slave_id(soil_modbus_rtu_ctx_t *ctx, uint8_t *slave_id_out)
{
    uint16_t reg = 0;
    soil_modbus_status_t status;

    if (slave_id_out == NULL)
    {
        return SOIL_MODBUS_STATUS_BAD_ARG;
    }

    status = soil_modbus_rtu_read_holding_registers(ctx, SOIL_MODBUS_BROADCAST_ADDRESS, SOIL_THC_REG_SLAVE_ID, 1U, &reg);
    if (status != SOIL_MODBUS_STATUS_OK)
    {
        return status;
    }

    if ((reg == 0U) || (reg > 254U))
    {
        return SOIL_MODBUS_STATUS_PROTOCOL;
    }

    *slave_id_out = (uint8_t)reg;
    return SOIL_MODBUS_STATUS_OK;
}

uint8_t soil_modbus_rtu_last_exception(const soil_modbus_rtu_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return 0U;
    }

    return ctx->last_exception;
}

const char *soil_modbus_status_to_string(soil_modbus_status_t status)
{
    switch (status)
    {
        case SOIL_MODBUS_STATUS_OK:
            return "ok";
        case SOIL_MODBUS_STATUS_BAD_ARG:
            return "bad argument";
        case SOIL_MODBUS_STATUS_UART:
            return "uart error";
        case SOIL_MODBUS_STATUS_TIMEOUT:
            return "timeout";
        case SOIL_MODBUS_STATUS_CRC:
            return "crc mismatch";
        case SOIL_MODBUS_STATUS_PROTOCOL:
            return "protocol error";
        case SOIL_MODBUS_STATUS_EXCEPTION:
            return "modbus exception";
        default:
            return "unknown";
    }
}
