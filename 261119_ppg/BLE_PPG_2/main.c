/*
 * BLE_PPG_2 — BLE PPG Monitor for nRF52840 + MAX30102
 *
 * Hardware:
 *   ProMicro nRF52840 + MH-ET LIVE MAX30102 module
 *   VIN→VCC, SDA→P1.13(or P1.11), SCL→P1.11(or P1.13), GND→GND
 *   P0.13 controls VCC power to MAX30102
 *
 * Features:
 *   1. VDD set to 3.3V (UICR REGOUT0)
 *   2. P0.13 = HIGH to power MAX30102
 *   3. Read PPG at 1000 Hz (no averaging), BLE notification output
 *   4. Advertising: LED blinks 1/sec; Connected: LED blinks 2/sec
 *   5. BLE packet: 19 bytes = 1 seq + 3 pairs of (RED 3B + IR 3B)
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "ble_conn_state.h"
#include "fstorage.h"
#include "fds.h"
#include "peer_manager.h"

#define NRF_LOG_MODULE_NAME "PPG"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// ============================================================
// Pin definitions (ProMicro nRF52840)
// ============================================================
#define PIN_VCC_EN          13                // P0.13 — power control for MAX30102
#define PIN_LED             6                 // P0.06 — blue LED on ProMicro (active low)

// I2C pins — SDK12 doesn't define P1.xx, we use raw pin numbers
// P1.13 = 32+13 = 45, P1.11 = 32+11 = 43
#define PIN_SDA_A           45                // P1.13
#define PIN_SCL_A           43                // P1.11
#define PIN_SDA_B           43                // swapped
#define PIN_SCL_B           45                // swapped

// TWIM instance 0 base address
#define TWIM0_BASE          0x40003000UL

// ============================================================
// MAX30102 definitions
// ============================================================
#define MAX30102_ADDR       0x57

// Register addresses
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG      0x09
#define REG_SPO2_CONFIG      0x0A
#define REG_LED1_PA          0x0C   // RED LED current
#define REG_LED2_PA          0x0D   // IR LED current
#define REG_PILOT_PA         0x10
#define REG_MULTI_LED_CTRL1  0x11
#define REG_MULTI_LED_CTRL2  0x12
#define REG_TEMP_INT         0x1F
#define REG_TEMP_FRAC        0x20
#define REG_TEMP_CONFIG      0x21
#define REG_REV_ID           0xFE
#define REG_PART_ID          0xFF

// ============================================================
// BLE parameters
// ============================================================
#define DEVICE_NAME                 "BLE_PPG"
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE        64
#endif

#define APP_FEATURE_NOT_SUPPORTED   (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2)
#define CENTRAL_LINK_COUNT          0
#define PERIPHERAL_LINK_COUNT       1

#define APP_ADV_INTERVAL            160     // 100 ms (160 * 0.625)
#define APP_ADV_TIMEOUT_IN_SECONDS  0       // never timeout

#define APP_TIMER_PRESCALER         0
#define APP_TIMER_OP_QUEUE_SIZE     8

#define MIN_CONN_INTERVAL           MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   // 7.5 ms
#define MAX_CONN_INTERVAL           MSEC_TO_UNITS(15, UNIT_1_25_MS)    // 15 ms
#define SLAVE_LATENCY               0
#define CONN_SUP_TIMEOUT            MSEC_TO_UNITS(4000, UNIT_10_MS)

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define DEAD_BEEF                   0xDEADBEEF

// PPG custom service UUID (128-bit)
// PPG_SERVICE:  12345678-1234-5678-1234-56789ABCDEF0
// PPG_CHAR:     12345678-1234-5678-1234-56789ABCDEF1
#define PPG_SERVICE_UUID_BASE  {0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
                                0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12}
#define PPG_SERVICE_UUID       0xDEF0
#define PPG_CHAR_UUID          0xDEF1

// ============================================================
// PPG packet format
// ============================================================
#define SAMPLES_PER_PKT  10
#define PPG_PKT_LEN      (1 + SAMPLES_PER_PKT * 6)   // 61

// ============================================================
// Ring buffer for PPG samples
// ============================================================
#define PPG_BUF_SIZE  4096

typedef struct {
    uint32_t red;
    uint32_t ir;
} ppg_sample_t;

static ppg_sample_t  m_ppg_buf[PPG_BUF_SIZE];
static volatile uint16_t m_ppg_head = 0;
static volatile uint16_t m_ppg_tail = 0;

// ============================================================
// Module variables
// ============================================================
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static uint8_t  m_ppg_uuid_type;
static uint16_t m_ppg_char_handle_value;
static uint16_t m_ppg_char_handle_cccd;
static bool     m_notifications_enabled = false;
static uint8_t  m_tx_seq = 0;

static uint32_t m_sda_pin;
static uint32_t m_scl_pin;

APP_TIMER_DEF(m_led_timer);
APP_TIMER_DEF(m_ppg_read_timer);

static volatile bool m_led_state = false;
static bool m_connected = false;

// ============================================================
// TWIM (I2C) register-level driver
// ============================================================
// Since SDK12 doesn't support P1.xx pins natively, we write TWIM registers directly.

static volatile uint32_t * const TWIM0 = (volatile uint32_t *)TWIM0_BASE;

// TWIM register offsets (word index)
#define TWIM_TASKS_STARTRX    (0x000 / 4)
#define TWIM_TASKS_STARTTX    (0x008 / 4)
#define TWIM_TASKS_STOP       (0x014 / 4)
#define TWIM_EVENTS_STOPPED   (0x104 / 4)
#define TWIM_EVENTS_ERROR     (0x124 / 4)
#define TWIM_EVENTS_LASTRX    (0x15C / 4)
#define TWIM_EVENTS_LASTTX    (0x160 / 4)
#define TWIM_SHORTS           (0x200 / 4)
#define TWIM_ERRORSRC         (0x4C4 / 4)
#define TWIM_ENABLE           (0x500 / 4)
#define TWIM_PSEL_SCL         (0x508 / 4)
#define TWIM_PSEL_SDA         (0x50C / 4)
#define TWIM_FREQUENCY         (0x524 / 4)
#define TWIM_RXD_PTR          (0x534 / 4)
#define TWIM_RXD_MAXCNT       (0x538 / 4)
#define TWIM_RXD_AMOUNT       (0x53C / 4)
#define TWIM_TXD_PTR          (0x544 / 4)
#define TWIM_TXD_MAXCNT       (0x548 / 4)
#define TWIM_TXD_AMOUNT       (0x54C / 4)
#define TWIM_ADDRESS          (0x588 / 4)

#define TWIM_FREQ_400K        0x06680000UL

static void twim_init(uint32_t sda_pin, uint32_t scl_pin)
{
    // Configure pins as inputs with pull-up (I2C idle state)
    nrf_gpio_cfg(sda_pin, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(scl_pin, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);

    TWIM0[TWIM_ENABLE] = 0;  // disable first
    TWIM0[TWIM_PSEL_SCL] = scl_pin;
    TWIM0[TWIM_PSEL_SDA] = sda_pin;
    TWIM0[TWIM_FREQUENCY] = TWIM_FREQ_400K;
    TWIM0[TWIM_ADDRESS] = MAX30102_ADDR;
    TWIM0[TWIM_ENABLE] = 6;  // enable TWIM
}

static bool twim_tx(const uint8_t *data, uint32_t len)
{
    TWIM0[TWIM_TXD_PTR] = (uint32_t)data;
    TWIM0[TWIM_TXD_MAXCNT] = len;
    TWIM0[TWIM_RXD_MAXCNT] = 0;
    TWIM0[TWIM_SHORTS] = (1 << 9);  // LASTTX_STOP
    TWIM0[TWIM_EVENTS_STOPPED] = 0;
    TWIM0[TWIM_EVENTS_ERROR] = 0;
    TWIM0[TWIM_TASKS_STARTTX] = 1;

    uint32_t timeout = 100000;
    while (!TWIM0[TWIM_EVENTS_STOPPED] && --timeout);
    if (!timeout || TWIM0[TWIM_EVENTS_ERROR]) {
        TWIM0[TWIM_ERRORSRC] = TWIM0[TWIM_ERRORSRC]; // clear
        return false;
    }
    return true;
}

static bool twim_tx_rx(const uint8_t *tx_data, uint32_t tx_len,
                       uint8_t *rx_data, uint32_t rx_len)
{
    TWIM0[TWIM_TXD_PTR] = (uint32_t)tx_data;
    TWIM0[TWIM_TXD_MAXCNT] = tx_len;
    TWIM0[TWIM_RXD_PTR] = (uint32_t)rx_data;
    TWIM0[TWIM_RXD_MAXCNT] = rx_len;
    TWIM0[TWIM_SHORTS] = (1 << 7) | (1 << 12);  // LASTTX_STARTRX | LASTRX_STOP
    TWIM0[TWIM_EVENTS_STOPPED] = 0;
    TWIM0[TWIM_EVENTS_ERROR] = 0;
    TWIM0[TWIM_TASKS_STARTTX] = 1;

    uint32_t timeout = 100000;
    while (!TWIM0[TWIM_EVENTS_STOPPED] && --timeout);
    if (!timeout || TWIM0[TWIM_EVENTS_ERROR]) {
        TWIM0[TWIM_ERRORSRC] = TWIM0[TWIM_ERRORSRC];
        return false;
    }
    return true;
}

// ============================================================
// MAX30102 register access
// ============================================================
static bool max30102_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return twim_tx(buf, 2);
}

static bool max30102_read_reg(uint8_t reg, uint8_t *val)
{
    return twim_tx_rx(&reg, 1, val, 1);
}

static bool max30102_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return twim_tx_rx(&reg, 1, buf, len);
}

// ============================================================
// MAX30102 initialization & auto-detect I2C wiring
// ============================================================
static bool max30102_probe(void)
{
    uint8_t part_id = 0;
    if (max30102_read_reg(REG_PART_ID, &part_id) && part_id == 0x15) {
        return true;
    }
    return false;
}

static bool max30102_init(void)
{
    // Try SDA=P1.13, SCL=P1.11 first
    twim_init(PIN_SDA_A, PIN_SCL_A);
    nrf_delay_ms(5);
    if (max30102_probe()) {
        m_sda_pin = PIN_SDA_A;
        m_scl_pin = PIN_SCL_A;
        NRF_LOG_INFO("I2C: SDA=P1.13, SCL=P1.11\r\n");
        goto configure;
    }

    // Try swapped
    twim_init(PIN_SDA_B, PIN_SCL_B);
    nrf_delay_ms(5);
    if (max30102_probe()) {
        m_sda_pin = PIN_SDA_B;
        m_scl_pin = PIN_SCL_B;
        NRF_LOG_INFO("I2C: SDA=P1.11, SCL=P1.13\r\n");
        goto configure;
    }

    NRF_LOG_ERROR("MAX30102 not found!\r\n");
    return false;

configure:
    // Reset
    max30102_write_reg(REG_MODE_CONFIG, 0x40);
    nrf_delay_ms(50);

    // Clear FIFO pointers
    max30102_write_reg(REG_FIFO_WR_PTR, 0x00);
    max30102_write_reg(REG_OVF_COUNTER, 0x00);
    max30102_write_reg(REG_FIFO_RD_PTR, 0x00);

    // Interrupts: A_FULL enable
    max30102_write_reg(REG_INTR_ENABLE_1, 0x80);
    max30102_write_reg(REG_INTR_ENABLE_2, 0x00);

    // FIFO config: sample avg = 1 (000), fifo_rollover_en, fifo_a_full = 17 (0x0F = 15 unread)
    max30102_write_reg(REG_FIFO_CONFIG, 0x0F);

    // Mode: SpO2 mode (0x03)
    max30102_write_reg(REG_MODE_CONFIG, 0x03);

    // SpO2 config (datasheet Table 11): ADC_RGE 11 (16384 nA), SR 101 (1000 sps), PW 01 (118 us / 16-bit)
    // 0b0_11_101_01 = 0x75 -> 1000 sps / 1 avg = 1000 Hz output
    max30102_write_reg(REG_SPO2_CONFIG, 0x75);

    // LED current: 7.2 mA = 0x24
    max30102_write_reg(REG_LED1_PA, 0x24);  // RED
    max30102_write_reg(REG_LED2_PA, 0x24);  // IR

    // Clear interrupt status
    uint8_t dummy;
    max30102_read_reg(REG_INTR_STATUS_1, &dummy);
    max30102_read_reg(REG_INTR_STATUS_2, &dummy);

    NRF_LOG_INFO("MAX30102 configured: SpO2 mode, 1000Hz (no avg), 118us\r\n");
    return true;
}

// ============================================================
// Read FIFO samples from MAX30102
// ============================================================
static uint32_t max30102_read_fifo(void)
{
    uint8_t wr_ptr, rd_ptr;
    max30102_read_reg(REG_FIFO_WR_PTR, &wr_ptr);
    max30102_read_reg(REG_FIFO_RD_PTR, &rd_ptr);

    int num_samples = (int)wr_ptr - (int)rd_ptr;
    if (num_samples < 0) num_samples += 32;
    if (num_samples == 0) return 0;

    uint32_t count = 0;
    for (int i = 0; i < num_samples; i++) {
        uint8_t buf[6];
        if (!max30102_read_regs(REG_FIFO_DATA, buf, 6)) break;

        uint32_t red = ((uint32_t)(buf[0] & 0x03) << 16) |
                       ((uint32_t)buf[1] << 8) | buf[2];
        uint32_t ir  = ((uint32_t)(buf[3] & 0x03) << 16) |
                       ((uint32_t)buf[4] << 8) | buf[5];

        uint16_t next = (m_ppg_head + 1) % PPG_BUF_SIZE;
        if (next != m_ppg_tail) {
            m_ppg_buf[m_ppg_head].red = red;
            m_ppg_buf[m_ppg_head].ir  = ir;
            m_ppg_head = next;
        }
        count++;
    }
    return count;
}

// ============================================================
// BLE: PPG custom service
// ============================================================
static uint32_t ppg_service_init(void)
{
    uint32_t err_code;
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = {PPG_SERVICE_UUID_BASE};

    err_code = sd_ble_uuid_vs_add(&base_uuid, &m_ppg_uuid_type);
    APP_ERROR_CHECK(err_code);

    service_uuid.type = m_ppg_uuid_type;
    service_uuid.uuid = PPG_SERVICE_UUID;

    uint16_t service_handle;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                         &service_uuid, &service_handle);
    APP_ERROR_CHECK(err_code);

    // Add PPG data characteristic (notify only)
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_cccd_md = &cccd_md;

    char_uuid.type = m_ppg_uuid_type;
    char_uuid.uuid = PPG_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&attr, 0, sizeof(attr));
    attr.p_uuid    = &char_uuid;
    attr.p_attr_md = &attr_md;
    attr.init_len  = 0;
    attr.init_offs = 0;
    attr.max_len   = PPG_PKT_LEN;

    ble_gatts_char_handles_t char_handles;
    err_code = sd_ble_gatts_characteristic_add(service_handle,
                                                &char_md, &attr, &char_handles);
    APP_ERROR_CHECK(err_code);

    m_ppg_char_handle_value = char_handles.value_handle;
    m_ppg_char_handle_cccd  = char_handles.cccd_handle;

    return NRF_SUCCESS;
}

// Send PPG data via BLE notification
// Packet format: [seq(1)] + 3 * [RED(3) + IR(3)] = 19 bytes
static void ppg_send_notification(void)
{
    if (!m_notifications_enabled || m_conn_handle == BLE_CONN_HANDLE_INVALID)
        return;

    uint16_t available = (m_ppg_head - m_ppg_tail + PPG_BUF_SIZE) % PPG_BUF_SIZE;
    while (available >= SAMPLES_PER_PKT) {
        uint8_t pkt[PPG_PKT_LEN];
        pkt[0] = m_tx_seq;

        for (int i = 0; i < SAMPLES_PER_PKT; i++) {
            ppg_sample_t s = m_ppg_buf[(m_ppg_tail + i) % PPG_BUF_SIZE];

            pkt[1 + i*6 + 0] = (s.red >> 16) & 0x03;
            pkt[1 + i*6 + 1] = (s.red >> 8)  & 0xFF;
            pkt[1 + i*6 + 2] = (s.red)       & 0xFF;
            pkt[1 + i*6 + 3] = (s.ir >> 16)  & 0x03;
            pkt[1 + i*6 + 4] = (s.ir >> 8)   & 0xFF;
            pkt[1 + i*6 + 5] = (s.ir)        & 0xFF;
        }

        uint16_t len = PPG_PKT_LEN;
        ble_gatts_hvx_params_t hvx;
        memset(&hvx, 0, sizeof(hvx));
        hvx.handle = m_ppg_char_handle_value;
        hvx.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx.p_len  = &len;
        hvx.p_data = pkt;

        uint32_t err = sd_ble_gatts_hvx(m_conn_handle, &hvx);
        if (err != NRF_SUCCESS) break;

        m_tx_seq++;
        m_ppg_tail = (m_ppg_tail + SAMPLES_PER_PKT) % PPG_BUF_SIZE;
        available = (m_ppg_head - m_ppg_tail + PPG_BUF_SIZE) % PPG_BUF_SIZE;
    }
}

// ============================================================
// VDD 3.3V setup (UICR REGOUT0)
// ============================================================
static void vdd_3v3_init(void)
{
    if ((NRF_UICR->REGOUT0 & 0x07) != 5) {  // 5 = 3.3V
        NRF_NVMC->CONFIG = 1;  // write enable
        while (!NRF_NVMC->READY);
        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~0x07UL) | 5;
        while (!NRF_NVMC->READY);
        NRF_NVMC->CONFIG = 0;
        NVIC_SystemReset();  // reset to apply
    }
}

// ============================================================
// LED blink timer handler
// ============================================================
static void led_timer_handler(void * p_context)
{
    m_led_state = !m_led_state;
    if (m_led_state) {
        nrf_gpio_pin_clear(PIN_LED);  // LED on (active low)
    } else {
        nrf_gpio_pin_set(PIN_LED);    // LED off
    }
}

static void led_set_blink_rate(bool connected)
{
    app_timer_stop(m_led_timer);
    uint32_t interval;
    if (connected) {
        interval = APP_TIMER_TICKS(250, APP_TIMER_PRESCALER);  // 2 Hz blink = 250ms toggle
    } else {
        interval = APP_TIMER_TICKS(500, APP_TIMER_PRESCALER);  // 1 Hz blink = 500ms toggle
    }
    app_timer_start(m_led_timer, interval, NULL);
}

// ============================================================
// PPG read timer handler (every 10ms = 100 Hz read, ~10 samples each)
// ============================================================
static void ppg_read_timer_handler(void * p_context)
{
    max30102_read_fifo();
    ppg_send_notification();
}

// ============================================================
// BLE event handler
// ============================================================
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            m_connected = true;
            led_set_blink_rate(true);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_connected = false;
            m_notifications_enabled = false;
            led_set_blink_rate(false);
            break;

        case BLE_GATTS_EVT_WRITE:
        {
            ble_gatts_evt_write_t * p_write = &p_ble_evt->evt.gatts_evt.params.write;
            if (p_write->handle == m_ppg_char_handle_cccd && p_write->len == 2) {
                m_notifications_enabled = (p_write->data[0] & 0x01);
                NRF_LOG_INFO("Notifications enabled=%d\r\n",
                             (uint32_t)m_notifications_enabled);
            }
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            break;

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                            NRF_BLE_MAX_MTU_SIZE);
            break;
#endif

        default:
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    fs_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

// ============================================================
// BLE stack init
// ============================================================
static void ble_stack_init(void)
{
    uint32_t err_code;
    // Internal RC + periodic calibration: works whether or not the board has a 32 kHz crystal
    nrf_clock_lf_cfg_t clock_lf_cfg = {
        .source        = NRF_CLOCK_LF_SRC_RC,
        .rc_ctiv       = 16,
        .rc_temp_ctiv  = 2,
        .xtal_accuracy = 0
    };

    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                     PERIPHERAL_LINK_COUNT,
                                                     &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

// ============================================================
// GAP, advertising, conn params, peer manager
// ============================================================
static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                           (const uint8_t *)DEVICE_NAME,
                                           strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Advertising\r\n");
            break;
        case BLE_ADV_EVT_IDLE:
            // Restart advertising (never sleep)
            ble_advertising_start(BLE_ADV_MODE_FAST);
            break;
        default:
            break;
    }
}

static ble_uuid_t m_adv_uuids[] = {{PPG_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};

static void advertising_init(void)
{
    uint32_t err_code;
    ble_advdata_t advdata;
    ble_adv_modes_config_t options;

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void pm_evt_handler(pm_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            ble_advertising_start(BLE_ADV_MODE_FAST);
            break;
        case PM_EVT_STORAGE_FULL:
            fds_gc();
            break;
        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
            break;
        default:
            break;
    }
}

static void peer_manager_init(void)
{
    uint32_t err_code;
    ble_gap_sec_params_t sec_param;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(sec_param));
    sec_param.bond           = 0;  // no bonding needed
    sec_param.mitm           = 0;
    sec_param.io_caps        = BLE_GAP_IO_CAPS_NONE;
    sec_param.min_key_size   = 7;
    sec_param.max_key_size   = 16;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

// ============================================================
// main
// ============================================================
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

int main(void)
{
    uint32_t err_code;

    // Set VDD to 3.3V (resets if changed)
    vdd_3v3_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    // Init app timer
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ppg_read_timer, APP_TIMER_MODE_REPEATED, ppg_read_timer_handler);
    APP_ERROR_CHECK(err_code);

    // LED init
    nrf_gpio_cfg_output(PIN_LED);
    nrf_gpio_pin_set(PIN_LED);  // LED off initially

    // Power on MAX30102 via P0.13
    nrf_gpio_cfg_output(PIN_VCC_EN);
    nrf_gpio_pin_set(PIN_VCC_EN);
    nrf_delay_ms(50);

    // BLE init
    ble_stack_init();
    peer_manager_init();
    gap_params_init();
    advertising_init();
    ppg_service_init();
    conn_params_init();

    // Init MAX30102
    if (!max30102_init()) {
        NRF_LOG_ERROR("MAX30102 init failed!\r\n");
        // Blink fast to indicate error
        while (1) {
            nrf_gpio_pin_toggle(PIN_LED);
            nrf_delay_ms(100);
        }
    }

    // Start LED blink (advertising rate)
    led_set_blink_rate(false);

    // Start PPG reading timer (every 10ms)
    err_code = app_timer_start(m_ppg_read_timer,
                                APP_TIMER_TICKS(10, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

    // Start advertising
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("BLE_PPG started\r\n");

    for (;;) {
        if (NRF_LOG_PROCESS() == false) {
            err_code = sd_app_evt_wait();
            APP_ERROR_CHECK(err_code);
        }
    }
}
