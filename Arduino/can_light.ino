#include <cstring>
#include <NativeEthernet.h>
#include <PubSubClient.h>
#include "gestureRecognizer.h"

#include "ODriveCAN.h"
#include <Wire.h>

#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus;

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

#define ENABLE_ODRIVE 0

#define CAN_BAUDRATE 250000

#define AXIS_0_ID 0
#define AXIS_1_ID 1

// SETTINGS
#define KEEP_ALIVE_MS 1000
#define KEEP_DEAD_MS 1500
#define LED_PIN 13
#define X_CENTER_OFFSET -0.280f
#define Y_CENTER_OFFSET -0.140f

// MQTT
byte NETWORK_MAC[] = { 0xDE, 0xAA, 0xBE, 0xEF, 0xF3, 0x08 };
IPAddress NETWORK_IP   (192, 168, 1, 31);
IPAddress serv         (192, 168, 1, 235);
const int  MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "Wand";

EthernetClient eth_Client;
PubSubClient MQTT_Client(eth_Client);
long lastReconnectAttempt = 0;

const Pos SpellPoints[] = {
    Pos(212, 212),
    Pos(335, 164),
    Pos(443, 253),
    Pos(460, 381),
    Pos(312, 412),
    Pos(172, 400),
    Pos(320, 300)
};

const int NUM_SPELL_POINTS = sizeof(SpellPoints) / (sizeof(Pos));
// GestureRecognizer recognizer(SpellPoints, NUM_SPELL_POINTS, 40.0f);
bool results[NUM_SPELL_POINTS];
// bool prev_results[NUM_SPELL_POINTS];

GestureHistory guesture_history(SpellPoints, NUM_SPELL_POINTS, 40.0f);
int history[MAX_CHECKPOINTS];
int prev_history[MAX_CHECKPOINTS];

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

int IRsensorAddress = 0xB0;
int slaveAddress;
bool ledState = false;
Pos wand_pos;
uint32_t time_killed;

byte data_buf[16];

bool light = false;

int Ix[4];
int Iy[4];

class ringBuffer
{
  int size {15};
  Pos data[15];
  int index {0};
  
  public:
    void append(Pos pos)
    {
      data[index % size] = pos;
      index ++;
    }

    void reset()
    {
      index = 0;
    }

    bool check_if_all_pos_are_with_in_a_small_group(int threshold)
    {
      // Check if the buffer is full
      if (index < size)
      {
        return false;
      }
      
      // find average of all the points
      Pos average = calculate_avg();

      double Dx;
      double Dy;
      for ( int i = 0; i < size; i++ )
      {
        Dx = average.x - data[i].x;
        Dy = average.y - data[i].y;
        double displacment = sqrt((Dx*Dx) + (Dy*Dy));
        
        if (displacment > threshold)
        {
          return false;
        }
      }
      return true;
    }
  private:
    Pos calculate_avg()
    {
      double sum_x = 0;
      double sum_y = 0;
      for (int i = 0; i < size; i++ )
      {
        sum_x += data[i].x;
        sum_y += data[i].y;
      }

      Pos avg;
      avg.x = sum_x / size;
      avg.y = sum_y / size;

      return avg;
    }
};

void Write_2bytes(byte d1, byte d2)
{
    Wire2.beginTransmission(slaveAddress);
    Wire2.write(d1); Wire2.write(d2);
    Wire2.endTransmission();
}

bool determine_detection(Pos pos)
{
    static uint32_t last_detected;
    static bool detected;
    if (!((pos.x == 1023) || (pos.y == 1023)))
    {
      last_detected = millis();
      detected = true;
      wand_pos = pos;
    }
    else if ( detected && ((millis() - last_detected) > KEEP_ALIVE_MS))
    {
      detected = false;
    }
    return detected;
}

ringBuffer buffer;

const int AXIS_0 = 0;
const int AXIS_1 = 1;

// ODRIVE
ODriveCAN axis0(wrap_can_intf(can_intf), AXIS_0_ID);
ODriveCAN axis1(wrap_can_intf(can_intf), AXIS_1_ID);

ODriveCAN* odrives[] = { &axis0, &axis1 };  // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData axis0_user_data;
ODriveUserData axis1_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

void send_light_mqtt(const bool state) {
  String message = "{\"state\": " + String(state) + " }";

  int   ArrayLength  = message.length()+1;
  char  CharArray[ArrayLength];
  message.toCharArray(CharArray, ArrayLength);

  MQTT_Client.publish("wand/state/light", CharArray);
}

void send_results_mqtt() {
  int state = 0;
  for(int i = 0; i < NUM_SPELL_POINTS; i++){
      state += results[i];
  }

  String message = "{\"state\": " + String(state) + " }";
  int   ArrayLength  = message.length()+1;
  char  CharArray[ArrayLength];
  message.toCharArray(CharArray, ArrayLength);

  MQTT_Client.publish("wand/state/checkpoint", CharArray);
}

void send_history_mqtt() {
  String message = "{\"history\": [";
  for (int i = 0; i < MAX_CHECKPOINTS; i++) {
    message += String(history[i]);
    if (i < MAX_CHECKPOINTS - 1) {
      message += ", ";
    }
  }
  message += "]}";

  int   ArrayLength  = message.length()+1;
  char  CharArray[ArrayLength];
  message.toCharArray(CharArray, ArrayLength);

  MQTT_Client.publish("wand/state/history", CharArray);
}

bool send_light_cmd(const bool state) {
  Set_gpio_msg_t msg;

  msg.request_type = 1;
  msg.gpio_num = 3;
  msg.state = state;

  Serial.print("light cmd = ");
  Serial.println(state);
  // publish state over mqtt
  send_light_mqtt(state);
  return axis0.send(msg);
}

void setup_light() {
  Set_gpio_msg_t msg;

  msg.request_type = 2;
  msg.gpio_num = 3;
  msg.state = 1;

  axis0.send(msg);
}

void setup_odrive() {
  // Register callbacks for the heartbeat and encoder feedback messages
  axis0.onFeedback(onFeedback, &axis0_user_data);
  axis0.onStatus(onHeartbeat, &axis0_user_data);
  axis1.onFeedback(onFeedback, &axis1_user_data);
  axis1.onStatus(onHeartbeat, &axis1_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true)
      ;  // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!axis0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("found ODrive");
  
  pumpEvents(can_intf);

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!axis0.request(vbus, 10)) {
    Serial.println("vbus request failed!");
    while (true)
      ;  // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Setting gpio 3 on ODrive to output...");
  setup_light();

  // Serial.println("Waiting for axis to be IDLE...");

  // delay(1000);
  // axis0.setState(ODriveAxisState::AXIS_STATE_IDLE);
  // axis1.setState(ODriveAxisState::AXIS_STATE_IDLE);

  // while (axis0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE && axis1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) {
  //   delay(1);

  //   for (int i = 0; i < 100; ++i) {
  //     delay(1);
  //     pumpEvents(can_intf);
  //   }
  // }

  // Serial.println("Both axis are IDLE...");
  // delay(100);
  // Serial.println("Waiting for axis to be ENCODER_INDEX_SEARCH...");

  // while (axis0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_ENCODER_INDEX_SEARCH && axis1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_ENCODER_INDEX_SEARCH) {
  //   delay(1);
  //   axis0.setState(ODriveAxisState::AXIS_STATE_ENCODER_INDEX_SEARCH);
  //   axis1.setState(ODriveAxisState::AXIS_STATE_ENCODER_INDEX_SEARCH);

  //   for (int i = 0; i < 100; ++i) {
  //     delay(1);
  //     pumpEvents(can_intf);
  //   }
  // }

  // Serial.println("Both axis are ENCODER_INDEX_SEARCH...");
  // delay(1000);
  // Serial.println("Waiting for axis to be IDLE...");

  // while (axis0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE && axis1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) {
  //   delay(1);

  //   for (int i = 0; i < 100; ++i) {
  //     delay(1);
  //     pumpEvents(can_intf);
  //   }
  // }

  // Serial.println("Both axis are IDLE...");
  // delay(1000);
  // Serial.println("Waiting for axis to be CLOSED_LOOP_CONTROL...");

  // while (axis0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL && axis1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
  //   delay(1);
  //   axis0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  //   axis1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

  //   for (int i = 0; i < 100; ++i) {
  //     delay(1);
  //     pumpEvents(can_intf);
  //   }
  // }
  // Serial.println("Both axis are CLOSED_LOOP_CONTROL...");
}

void setup_ir_sensor() {
  slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
  Wire2.begin();
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10);
  Write_2bytes(0x33,0x33); delay(10);
}

void setup_mqtt() {
  Serial.println("\n Setting up Ethernet: ");

  Ethernet.begin(NETWORK_MAC, NETWORK_IP); 
  Serial.println("\nEthernet connected with IP address: ");
  Serial.println(Ethernet.localIP());

  MQTT_Client.setServer(serv, MQTT_PORT);
}

bool reconnect_mqtt() {
  if (MQTT_Client.connect(MQTT_CLIENT_ID, "Mqtt", "Kacper")) {
    // Once connected, publish an announcement...
    // client.publish("outTopic","hello world");
    // ... and resubscribe
    // client.subscribe("inTopic");
  }
  return MQTT_Client.connected();
}

void setup() {
  delay(1000);

  Serial.begin(115200);

  setup_ir_sensor();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);

  setup_mqtt();

  setup_odrive();
  // recognizer.resetSpellProgress();
  guesture_history.resetSpellHistory();
  memset(history, -1, sizeof(history));
  memset(prev_history, -1, sizeof(prev_history));
}

void blink_the_led() {
  ledState = !ledState;
  if (ledState) { digitalWrite(LED_PIN,HIGH); } else { digitalWrite(LED_PIN,LOW); }
}

void read_ir_sensor() {
  //IR sensor read
  static int s;
  Wire2.beginTransmission(slaveAddress);
  Wire2.write(0x36);
  Wire2.endTransmission();

  Wire2.requestFrom(slaveAddress, 16); // Request the 2 byte heading (MSB comes first)
  for (int i = 0; i < 16; i++)
  {
    data_buf[i] = 0; // Clear the data buffer
    if (Wire2.available())
    {
      data_buf[i] = Wire2.read();
    }
  }

  Ix[0] = data_buf[1];
  Iy[0] = data_buf[2];
  s   = data_buf[3];
  Ix[0] += (s & 0x30) <<4;
  Iy[0] += (s & 0xC0) <<2;

  Ix[1] = data_buf[4];
  Iy[1] = data_buf[5];
  s   = data_buf[6];
  Ix[1] += (s & 0x30) <<4;
  Iy[1] += (s & 0xC0) <<2;

  Ix[2] = data_buf[7];
  Iy[2] = data_buf[8];
  s   = data_buf[9];
  Ix[2] += (s & 0x30) <<4;
  Iy[2] += (s & 0xC0) <<2;

  Ix[3] = data_buf[10];
  Iy[3] = data_buf[11];
  s   = data_buf[12];
  Ix[3] += (s & 0x30) <<4;
  Iy[3] += (s & 0xC0) <<2;
}

void loop()
{
  blink_the_led();

  // Check mqtt connection
  if (!MQTT_Client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect_mqtt()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    MQTT_Client.loop();
  }

  read_ir_sensor();

  bool detected = determine_detection(Pos(Ix[0], Iy[0]));
  
  if (detected)
  {
    buffer.append(wand_pos);

    if (buffer.check_if_all_pos_are_with_in_a_small_group(10) &&
        !light && 
        (millis() - time_killed) > KEEP_DEAD_MS)
    {
      light = true;
      send_light_cmd(light);
    }
  }
  else
  {
    buffer.reset();
    if (light)
    {
      light = false;
      time_killed = millis();
      send_light_cmd(light);
    }
  }

    // Serial.print( Ix[0] );
    // Serial.print(",");
    // Serial.print( Iy[0] );
    // Serial.print(",");
    // Serial.print(detected);
    // Serial.print(",");
    // Serial.print(Fx);
    // Serial.print(",");
    // Serial.print(Fy);
    // Serial.print(",");
    // Serial.println(light);


  // uint16_t cmd_X = constrain((Ix[0] - center_offset.y), -800, 800);
  // uint16_t cmd_y = constrain((Iy[0] - center_offset.x), -800, 800);

  // Serial.print(cmd_X);
  // Serial.print(", ");
  // Serial.print(cmd_y);
  // Serial.print("\n");


  // Check spell progress
  // recognizer.checkSpellProgress(wand_pos, light, results);
  guesture_history.checkSpellProgress(wand_pos, light);

  guesture_history.getCheckpointHistory(history, MAX_CHECKPOINTS);

  if (memcmp(&history, &prev_history, sizeof(history)) != 0)
  {
    Serial.printf("Sending mqtt message");
    memcpy(&prev_history, &history, sizeof(history));
    send_history_mqtt();
  }

  if (light)
  {
    float distance_from_wall = 300;

    uint16_t cmd_X = constrain(wand_pos.x, -800, 800);
    uint16_t cmd_Y = constrain(wand_pos.y, -800, 800);

    float cmd_x_f = static_cast<float>(cmd_X);
    float cmd_y_f = static_cast<float>(cmd_Y);

    float x_cm = constrain(map(cmd_x_f, -800.0f, 800.0f, -20.0f, 20.0f), -20.0f, 20.0f);
    float y_cm = constrain(map(cmd_y_f, -800.0f, 800.0f, -20.0f, 20.0f), -20.0f, 20.0f);

    float angle_x_unconstrained = atan(x_cm / distance_from_wall);
    float angle_y_unconstrained = atan(y_cm / distance_from_wall);

    float max_throw = 0.05;

    float angle_x = constrain(angle_x_unconstrained, -max_throw, max_throw) + X_CENTER_OFFSET;
    float angle_y = constrain(angle_y_unconstrained, -max_throw, max_throw) + Y_CENTER_OFFSET;

    Serial.printf("%f, %f\t", cmd_x_f, cmd_y_f);
    for(int i = 0; i < MAX_CHECKPOINTS; i++){
        Serial.print(history[i]);
        Serial.print(" ");
    }
    Serial.println();

    // if x and y are the position of the wand tip
    // and the light is a distance away form the wall
    // tan(angle) = x / a so atan(x/a) = angle_x
    // tan(angle) = y / a so atan(y/a) = angle_y
    
    axis0.setPosition(angle_y); // center -0.28  throw 0.05
    axis1.setPosition(angle_x); // center -0.135 throw 0.05
  }

  pumpEvents(can_intf);

  // float SINE_PERIOD = 3.0f; // Period of the position command sine wave in seconds

  // float t = 0.001 * millis();

  // float phase_x = ((cos(t * (TWO_PI / SINE_PERIOD)) + 1.0f)/2.0f)*65535.0;
  // float phase_y = ((sin(t * (TWO_PI / SINE_PERIOD)) + 1.0f)/2.0f)*65535.0;

  // Serial.print(phase_x);
  // Serial.print(",");
  // Serial.println(phase_y);

  // float drv_x_f = static_cast<float>(phase_x);
  // float drv_y_f = static_cast<float>(phase_y);

  // float drv_x_f = static_cast<float>(cmd.x);
  // float drv_y_f = static_cast<float>(cmd.y);

  // float drv_x = constrain(map(drv_x_f, 0.0f, 65535.0f, -5.0f, 5.0f), -5.0f, 5.0f) * -1.0f;
  // float drv_y = constrain(map(drv_y_f, 0.0f, 65535.0f, -5.0f, 5.0f), -5.0f, 5.0f);

  // float rotation_x = (drv_x / 100.0f) + center_offset.x;
  // float rotation_y = (drv_y / 100.0f) + center_offset.y;

  // Serial.printf("%f, %f\n", rotation_x, rotation_y);

  // axis0.setPosition(
  //   sin(rotation_x));

  // axis1.setPosition(
  //   sin(rotation_y));
  
  delay(1);
}