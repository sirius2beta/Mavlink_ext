#include <Arduino.h>
#include <MAVLink.h>

/*
 * board: ESP32C3 supermini
 */

HardwareSerial SerialTaiRa(1);
HardwareSerial SerialDev(2);

/* ---------- UART 設定 ---------- */
#define UART1_BAUD 115200
#define UART2_BAUD 115200
#define UART1_RX_PIN 8
#define UART1_TX_PIN 9
#define UART2_RX_PIN 4
#define UART2_TX_PIN 2

/* ---------- FreeRTOS Queue ---------- */
QueueHandle_t q_uart1;
QueueHandle_t q_uart2;

/* ---------- 狀態機 ---------- */
typedef enum { STATE_PARSE_MAVLINK, STATE_READ_EXTRA } rx_state_t;

typedef struct {
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t extra_buf[3];
  uint8_t extra_index;
  rx_state_t state;
} mav_bridge_t;

mav_bridge_t u1_to_u2;
mav_bridge_t u2_to_u1;

/* ---------- RSSI / Error ---------- */
volatile uint8_t rssi_rx = 0;
volatile uint8_t rssi_tx = 0;
volatile uint8_t err = 0;

void handle_extra_bytes(uint8_t *b) {
  rssi_tx = b[0];
  rssi_rx = b[1];
  err    = b[2];
}

/* ---------- Bridge Parser ---------- */
void bridge_process_byte(mav_bridge_t *b, uint8_t c, HardwareSerial &tx) {
  if (b->state == STATE_PARSE_MAVLINK) {
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &b->msg, &b->status)) {
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      uint16_t len = mavlink_msg_to_send_buffer(buf, &b->msg);
      tx.write(buf, len);

      b->state = STATE_READ_EXTRA;
      b->extra_index = 0;
    }
  } else { // STATE_READ_EXTRA
    b->extra_buf[b->extra_index++] = c;
    if (b->extra_index >= 3) {
      handle_extra_bytes(b->extra_buf);
      b->state = STATE_PARSE_MAVLINK;
    }
  }
}

void bridge_process_byte_v2(mav_bridge_t *b,
                         uint8_t c,
                         HardwareSerial &tx)
{
  if (mavlink_parse_char(MAVLINK_COMM_0, c,
                           &b->msg, &b->status)) {


      // 收到完整 MAVLink
      uint8_t buf[MAVLINK_MAX_PACKET_LEN+1];
      uint16_t len = mavlink_msg_to_send_buffer(buf, &b->msg);
      buf[len] = rssi_tx;
      tx.write(buf, len+1);


    }
}

/* ---------- UART RX Tasks ---------- */
void uart1_rx_task(void *arg) {
  uint8_t c;
  while (1) {
    if (SerialTaiRa.available()) {
      c = SerialTaiRa.read();
      xQueueSend(q_uart1, &c, portMAX_DELAY);
    } else {
      vTaskDelay(1);
    }
  }
}

void uart2_rx_task(void *arg) {
  uint8_t c;
  while (1) {
    if (SerialDev.available()) {
      c = SerialDev.read();
      xQueueSend(q_uart2, &c, portMAX_DELAY);
    } else {
      vTaskDelay(1);
    }
  }
}

/* ---------- Parser Task ---------- */
void parser_task(void *arg) {
  uint8_t c;
  while (1) {
    if (xQueueReceive(q_uart1, &c, 0) == pdTRUE)
      bridge_process_byte(&u1_to_u2, c, SerialDev);

    if (xQueueReceive(q_uart2, &c, 0) == pdTRUE)
      bridge_process_byte_v2(&u2_to_u1, c, SerialTaiRa);

    vTaskDelay(1);
  }
}

/* ---------- RSSI Print Task ---------- */
void rssi_print_task(void *arg) {
  while (1) {
    Serial.print("RSSI TX: "); Serial.print(rssi_tx);
    Serial.print(" RX: "); Serial.print(rssi_rx);
    Serial.print(" ERR: "); Serial.println(err);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);
  SerialTaiRa.begin(UART1_BAUD, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  SerialDev.begin(UART2_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

  u1_to_u2.state = STATE_PARSE_MAVLINK;
  u2_to_u1.state = STATE_PARSE_MAVLINK;

  q_uart1 = xQueueCreate(256, sizeof(uint8_t));
  q_uart2 = xQueueCreate(256, sizeof(uint8_t));

  xTaskCreatePinnedToCore(uart1_rx_task, "uart1_rx", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(uart2_rx_task, "uart2_rx", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(parser_task, "parser", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(rssi_print_task, "rssi_print", 2048, NULL, 1, NULL, 0);

  Serial.println("MAVLink UART bridge started");
}

/* ---------- Loop ---------- */
void loop() {
  // 空即可，所有工作都由 FreeRTOS tasks 完成
}
