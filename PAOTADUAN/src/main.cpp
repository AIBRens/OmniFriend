#include <Arduino.h>
#include <SimpleFOC.h>
#include "nRF24L01_API.h" 

// 强行声明 NRF 函数，防止找不到
extern void NRF24L01_Pin_Init(void);
extern unsigned char NRF24L01_Check(void);
extern void NRF24L01_RX_Init(void);
extern unsigned char NRF24L01_RxPacket(unsigned char *rxbuf);

// [NODE: I2C_INSTANCE_START]
struct MySoftI2C {
  uint8_t sda, scl;
  void init(uint8_t _sda, uint8_t _scl) {
    sda = _sda; scl = _scl;
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
  }
  void sda_out(bool val) { if(val) pinMode(sda, INPUT_PULLUP); else { pinMode(sda, OUTPUT); digitalWrite(sda, LOW); } }
  void scl_out(bool val) { if(val) pinMode(scl, INPUT_PULLUP); else { pinMode(scl, OUTPUT); digitalWrite(scl, LOW); } }
  bool sda_in() { return digitalRead(sda); }
  void dly() { delayMicroseconds(2); } 

  void start() { sda_out(1); scl_out(1); dly(); sda_out(0); dly(); scl_out(0); dly(); }
  void stop()  { sda_out(0); scl_out(0); dly(); scl_out(1); dly(); sda_out(1); dly(); }
  
  bool write(uint8_t data) {
    for(int i=0; i<8; i++) {
      sda_out(data & 0x80); dly(); scl_out(1); dly(); scl_out(0); dly();
      data <<= 1;
    }
    sda_out(1); dly(); scl_out(1); dly();
    bool ack = !sda_in(); 
    scl_out(0); dly();
    return ack;
  }
  
  uint8_t read(bool ack) {
    uint8_t data = 0;
    sda_out(1); 
    for(int i=0; i<8; i++) {
      data <<= 1; dly(); scl_out(1); dly();
      if(sda_in()) data |= 1;
      scl_out(0); dly();
    }
    sda_out(!ack); 
    dly(); scl_out(1); dly(); scl_out(0); dly();
    sda_out(1);
    return data;
  }
};

MySoftI2C i2c0;
MySoftI2C i2c1;
// [NODE: I2C_INSTANCE_END]

// [NODE: AI_UART_INSTANCE_START]
// 实例化原生硬件串口用于接收 AI 小智指令 (RX=PC11, TX=PC10)
HardwareSerial Serial_AI(PC11, PC10); 
// [NODE: AI_UART_INSTANCE_END]

// [NODE: CUSTOM_SENSOR_START]
float last_calculated_angle[2] = {0, 0};

float track_angle(uint16_t raw_adc, int sensor_idx) {
  static uint16_t last_raw[2] = {0, 0};
  static int32_t full_rotations[2] = {0, 0};
  static bool first_read[2] = {true, true};

  if (first_read[sensor_idx]) {
    last_raw[sensor_idx] = raw_adc;
    first_read[sensor_idx] = false;
    last_calculated_angle[sensor_idx] = (raw_adc / 4096.0f) * _2PI;
    return last_calculated_angle[sensor_idx];
  }

  int16_t d_raw = raw_adc - last_raw[sensor_idx];

  if (abs(d_raw) > 300 && abs(d_raw) < 3796) {
    return last_calculated_angle[sensor_idx]; 
  }

  if (d_raw > 2048)       full_rotations[sensor_idx]--;
  else if (d_raw < -2048) full_rotations[sensor_idx]++;

  last_raw[sensor_idx] = raw_adc;
  last_calculated_angle[sensor_idx] = ((full_rotations[sensor_idx] * 4096) + raw_adc) / 4096.0f * _2PI;
  return last_calculated_angle[sensor_idx];
}

float readSensor0() {
  i2c0.start();
  if(!i2c0.write(0x36 << 1)) { i2c0.stop(); return last_calculated_angle[0]; } 
  if(!i2c0.write(0x0C))      { i2c0.stop(); return last_calculated_angle[0]; }
  i2c0.stop();
  i2c0.start();
  if(!i2c0.write((0x36 << 1) | 1)) { i2c0.stop(); return last_calculated_angle[0]; }
  uint8_t hi = i2c0.read(true);
  uint8_t lo = i2c0.read(false);
  i2c0.stop();
  
  // ★ 神级修复：高位寄存器头 4 位必然是 0。如果不为 0 (如0xFF)，绝对是断线乱码！一击必杀！
  if ((hi & 0xF0) != 0) return last_calculated_angle[0];

  return track_angle(((hi << 8) | lo) & 0x0FFF, 0);
}
void initSensor0() { i2c0.init(PB11, PB10); }
GenericSensor sensor0 = GenericSensor(readSensor0, initSensor0);

float readSensor1() {
  i2c1.start();
  if(!i2c1.write(0x36 << 1)) { i2c1.stop(); return last_calculated_angle[1]; }
  if(!i2c1.write(0x0C))      { i2c1.stop(); return last_calculated_angle[1]; }
  i2c1.stop();
  i2c1.start();
  if(!i2c1.write((0x36 << 1) | 1)) { i2c1.stop(); return last_calculated_angle[1]; }
  uint8_t hi = i2c1.read(true);
  uint8_t lo = i2c1.read(false);
  i2c1.stop();
  
  // ★ 同样为 M1 加上这层物理防弹衣
  if ((hi & 0xF0) != 0) return last_calculated_angle[1];

  return track_angle(((hi << 8) | lo) & 0x0FFF, 1);
}
void initSensor1() { i2c1.init(PB9, PB8); }
GenericSensor sensor1 = GenericSensor(readSensor1, initSensor1);
// [NODE: CUSTOM_SENSOR_END]

// [NODE: MOTOR_INSTANCE_START]
BLDCMotor motor0 = BLDCMotor(11); 
BLDCDriver3PWM driver0 = BLDCDriver3PWM(PA8, PA9, PA10, NOT_SET);  

BLDCMotor motor1 = BLDCMotor(11); 
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA0, PA1, PA2, NOT_SET);  
// [NODE: MOTOR_INSTANCE_END]

// [NODE: GLOBAL_VARIABLES_START]
LowPassFilter filter_yaw(0.02);   
LowPassFilter filter_pitch(0.02); 

uint8_t rx_buf[32]; 

float yaw_offset = 0;   
float pitch_offset = 0; 
bool first_data_received = false; 

float initial_motor0_angle = 0;
float initial_motor1_angle = 0;

// ================= AI 小智开火控制：新增全局状态 =================
// ESP32 AI 小智会通过 UART2 发来单字节：'F' 开火，'S' 停火。
// 为了防止停火字节丢失，炮塔端收到 'F' 后本地最多保持 3000ms，然后自动停火。
static const uint8_t FIRE_PIN = PA3;
static const uint32_t AI_FIRE_DURATION_MS = 3000;

bool ai_fire_active = false;
uint32_t ai_fire_deadline_ms = 0;

void StartAIFire() {
  ai_fire_active = true;
  ai_fire_deadline_ms = millis() + AI_FIRE_DURATION_MS;
  Serial.println("AI Command: FIRE for 3 seconds.");
}

void StopAIFire() {
  if (ai_fire_active) {
    Serial.println("AI Command: STOP.");
  }
  ai_fire_active = false;
}

void UpdateAIFireTimeout() {
  if (ai_fire_active && (int32_t)(millis() - ai_fire_deadline_ms) >= 0) {
    ai_fire_active = false;
    Serial.println("AI Fire timeout: auto stop.");
  }
}
// ================================================================
// [NODE: GLOBAL_VARIABLES_END]

void setup() {
  // [NODE: SYSTEM_INIT_START]
  Serial.setTx(PB6);
  Serial.setRx(PB7);
  Serial.begin(115200);
  delay(1500);
  Serial.println("\r\n--- V3P Turret System (AI Voice & RF Dual Control) ---");  

  // 初始化 AI 小智监听串口 (波特率必须与 ESP32 源码中的 115200 一致)
  Serial_AI.begin(115200);

  // 唤醒 V3P 驱动板
  pinMode(PB12, OUTPUT);
  digitalWrite(PB12, HIGH);

  // 初始化开火控制 MOS 管引脚 PA3，默认低电平停火
  pinMode(FIRE_PIN, OUTPUT);
  digitalWrite(FIRE_PIN, LOW);
  // [NODE: SYSTEM_INIT_END]

  // [NODE: NRF_INIT_START]
  NRF24L01_Pin_Init();
  if (NRF24L01_Check() == 0) {
    Serial.println("NRF24L01 detected successfully!");
  } else {
    Serial.println("WARNING: NRF24L01 NOT found!");
  }
  NRF24L01_RX_Init(); 
  Serial.println("NRF24L01 Listening...");
  // [NODE: NRF_INIT_END]

  // [NODE: SENSOR_INIT_START]
  sensor0.init();
  motor0.linkSensor(&sensor0);
  
  sensor1.init();
  motor1.linkSensor(&sensor1);  
  // [NODE: SENSOR_INIT_END]

  // [NODE: DRIVER_INIT_START]
  driver0.voltage_power_supply = 14.8; 
  driver0.init();
  motor0.linkDriver(&driver0);  

  driver1.voltage_power_supply = 14.8; 
  driver1.init();
  motor1.linkDriver(&driver1);  
  // [NODE: DRIVER_INIT_END]

   // [NODE: FOC_PARAMS_START]
  motor0.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;

  motor0.voltage_limit = 7.0;
  motor1.voltage_limit = 7.0;
  motor0.velocity_limit = 20.0;  
  motor1.velocity_limit = 20.0;  
  
  motor0.voltage_sensor_align = 4.0;
  motor1.voltage_sensor_align = 4.0;
  motor0.motion_downsample = 2; 
  motor1.motion_downsample = 2; 
  motor0.LPF_velocity.Tf = 0.05;
  motor1.LPF_velocity.Tf = 0.05;

  motor0.P_angle.P = 12.0;
  motor1.P_angle.P = 12.0;
  motor0.PID_velocity.P = 0.5;
  motor1.PID_velocity.P = 0.5;
  
  motor0.PID_velocity.I = 2.0;  
  motor1.PID_velocity.I = 2.0;  

  motor0.PID_velocity.output_ramp = 1000.0; 
  motor1.PID_velocity.output_ramp = 1000.0; 
  
  motor0.PID_velocity.limit = 7.0; 
  motor1.PID_velocity.limit = 7.0;
  // [NODE: FOC_PARAMS_END]

   // [NODE: FOC_CALIBRATION_START]
  motor0.init();
  motor1.init();
  
  motor0.zero_electric_angle = 0.1427;
  motor0.sensor_direction = Direction::CW;
  
  motor1.zero_electric_angle = 2.4360;
  motor1.sensor_direction = Direction::CW;

  Serial.println("Skipping Auto-Calibration, using saved values...");
  
  motor0.initFOC();  
  motor1.initFOC();  

  initial_motor0_angle = sensor0.getAngle();
  initial_motor1_angle = sensor1.getAngle();
  
  motor0.target = initial_motor0_angle;
  motor1.target = initial_motor1_angle;

  Serial.println("\r\nSystem Ready! Waiting for Head Tracker data...");
  // [NODE: FOC_CALIBRATION_END]
}  

void loop() {
    // 用于安全断电的失联时间戳
 
  // [NODE: LOOP_RATE_LIMITER_START]
  static uint32_t last_loop_time = 0;
  if (micros() - last_loop_time < 2500) return; 
  last_loop_time = micros();
  // [NODE: LOOP_RATE_LIMITER_END]

  // [NODE: NRF_RECEIVE_START]
  // ================= 1. 处理 AI 小智语音开火指令 =================
  // AI 小智现有源码中 SendTurretFire(true) 发送 'F'，3秒后发送 'S'。
  // 这里不再做“偏航轴转动10度”的测试动作，只控制 PA3 开火输出。
  while (Serial_AI.available()) {
      char c = Serial_AI.read();
      if (c == 0x46 || c == 'F') {      // 'F' = FIRE
          StartAIFire();
      } else if (c == 0x53 || c == 'S') { // 'S' = STOP
          StopAIFire();
      }
  }

  // 本地3秒自动停火保护：即使 AI 小智的 'S' 字节丢失，也不会一直开火
  UpdateAIFireTimeout();

  // ================= 2. 处理头瞄物理按键指令 =================
  static uint32_t last_valid_rx_time = 0; 
  static bool rf_fire_active = false; 

  if (digitalRead(PC6) == LOW) { 
    if (NRF24L01_RxPacket(rx_buf) == 0) {
      if (rx_buf[0] == 5) { 
        
        last_valid_rx_time = millis(); 

        int16_t raw_pitch = (rx_buf[1] << 8) | rx_buf[2];
        int16_t raw_yaw   = (rx_buf[3] << 8) | rx_buf[4];
        uint8_t fire_cmd  = rx_buf[5]; 
        
        float current_pitch = raw_pitch / 100.0f;
        float current_yaw   = raw_yaw / 100.0f;

        static float last_valid_pitch = 0;
        static float last_valid_yaw = 0;
        static int reject_count = 0; 
        bool valid_packet = true;

        if (!first_data_received) {
            pitch_offset = current_pitch;
            yaw_offset = current_yaw;
            last_valid_pitch = current_pitch;
            last_valid_yaw   = current_yaw;
            first_data_received = true;
            Serial.println("First Head Tracker data received! Turret Synced.");
        } else {
            if (abs(current_pitch - last_valid_pitch) > 30.0f || abs(current_yaw - last_valid_yaw) > 30.0f) {
                reject_count++;
                if (reject_count < 5) valid_packet = false;
            }
            if (valid_packet) {
                reject_count = 0;
                last_valid_pitch = current_pitch;
                last_valid_yaw   = current_yaw;
            }
        }

        if (valid_packet) {
            float target_pitch_rad = -(current_pitch - pitch_offset) * (PI / 180.0f);
            float target_yaw_rad   = -(current_yaw - yaw_offset) * (PI / 180.0f);

            float pitch_limit_deg = 25.0f; 
            float yaw_limit_deg   = 85.0f; 
            
            float pitch_limit_rad = pitch_limit_deg * (PI / 180.0f);
            float yaw_limit_rad   = yaw_limit_deg * (PI / 180.0f);

            target_pitch_rad = constrain(target_pitch_rad, -pitch_limit_rad, pitch_limit_rad);
            target_yaw_rad   = constrain(target_yaw_rad, -yaw_limit_rad, yaw_limit_rad);

            motor1.target = filter_pitch(target_pitch_rad) + initial_motor1_angle; 
            motor0.target = filter_yaw(target_yaw_rad) + initial_motor0_angle;   

            // 提取无线开火状态
            if (fire_cmd == 1) {
                rf_fire_active = true;
            } else {
                rf_fire_active = false;
            }
        }
      }
    }
  }

  // ================= 3. 完美融合判定与安全控制 =================
  if (millis() - last_valid_rx_time > 200) {
      rf_fire_active = false; // 失联保护，仅清除物理按键开火状态
  }

  // 只要有一个要求开火，MOS 管就导通
  if (ai_fire_active || rf_fire_active) {
      digitalWrite(FIRE_PIN, HIGH);
  } else {
      digitalWrite(FIRE_PIN, LOW);
  }
  // [NODE: NRF_RECEIVE_END]

  // [NODE: FOC_LOOP_START]
  // ★ 静态死区算法：误差小于 0.02 弧度时切断动力。调用官方 reset() 方法安全清空积分
  if (abs(motor0.target - sensor0.getAngle()) < 0.02) {
      motor0.voltage_limit = 0; 
      motor0.PID_velocity.reset(); 
  } else {
      motor0.voltage_limit = 7.0; 
  }

  if (abs(motor1.target - sensor1.getAngle()) < 0.02) {
      motor1.voltage_limit = 0; 
      motor1.PID_velocity.reset(); 
  } else {
      motor1.voltage_limit = 7.0; 
  }

  motor0.loopFOC();
  motor0.move();  
  motor1.loopFOC();
  motor1.move();  
  // [NODE: FOC_LOOP_END]
}