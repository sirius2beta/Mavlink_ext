#include <MAVLink.h>
#include <Arduino.h>

#define UART2_TX 16
#define UART2_RX 17

HardwareSerial mySerial2(1);
void setup() {
  Serial.begin(115200);
  mySerial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
}

void loop() {
  // Send HEARTBEAT message to Serial once a second
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN + 3];

  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_STANDBY);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  buf[len] = 0x01;
  buf[len + 1] = 0x02;
  buf[len + 2] = 0x03;
  Serial.write(buf, len + 3);
  mySerial2.write(buf, len + 3);
  delay(1000);
}
