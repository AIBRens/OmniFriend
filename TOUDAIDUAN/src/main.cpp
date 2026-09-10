#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "nRF24L01_API.h" // 引入无线通信模块头文件

HardwareSerial Serial1(PA10, PA9);

#define AD0_VAL 0 
ICM_20948_I2C myICM;

// NRF24L01 发送缓冲区 (最大32字节)
uint8_t rece_buf[32]; 

void setup() {
    // 初始化发射按钮，使用内部下拉电阻！
  pinMode(PB12, INPUT_PULLDOWN);
  Serial1.begin(115200);
  delay(2000); 
  Serial1.println("\r\n====================================");
  Serial1.println("--- STM32 Head Tracker (Remapped Axis) ---");

  // ================= 1. 初始化 NRF24L01 =================
  Serial1.println("Initializing NRF24L01...");
  NRF24L01_Pin_Init();
  
  if (NRF24L01_Check() == 0) {
    Serial1.println("NRF24L01 detected successfully!");
  } else {
    Serial1.println("NRF24L01 NOT found! Check SPI wiring.");
  }
  
  // 配置为 2Mbps 且无自动应答的极限低延迟单向模式
  NRF24L01_RT_Init(); 

  // ================= 2. 初始化 I2C 与 陀螺仪 =================
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin(); 
  Wire.setClock(100000); // 100kHz 稳定模式
  Serial1.println("I2C configured. Initializing ICM20948...");

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial1.println("ICM20948 not detected! Please check wiring.");
      delay(1000);
    } else {
      initialized = true;
      Serial1.println("ICM20948 detected! Loading DMP Firmware...");
    }
  }

  // ================= 3. 启动 DMP =================
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) {
    Serial1.println("DMP initialized successfully! System is LIVE!");
  } else {
    Serial1.println("DMP initialization failed! System halted.");
    while(1); 
  }
}

void loop() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  // 记录初始零点的静态变量
  static bool is_zero_recorded = false;
  static uint32_t dmp_start_time = 0; 
  static double initial_pitch = 0.0;
  static double initial_roll  = 0.0;
  static double initial_yaw   = 0.0;

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    
    // 确认读到的是 6轴四元数 数据
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      
      // 1. 读取物理传感器的原始四元数数据
      // 你的新安装位置：X(右), Y(前), Z(上)
      double raw_q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // 物理 X
      double raw_q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // 物理 Y
      double raw_q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // 物理 Z

      // 2. 坐标轴重映射
      // 目标标准数学模型必须是：X(前)，Y(左)，Z(上)
      double q1 =  raw_q2;   // 数学 X(前) = 物理 Y(前)
      double q2 = -raw_q1;   // 数学 Y(左) = -物理 X(右)
      double q3 =  raw_q3;   // 数学 Z(上) = 物理 Z(上)
      
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      // 3. 计算绝对欧拉角 (此时 Yaw 和 Roll 的串轴已经被修复！)
      double abs_pitch = asin(2.0 * (q0 * q2 - q1 * q3)) * 180.0 / PI;
      double abs_roll  = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) * 180.0 / PI;
      double abs_yaw   = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)) * 180.0 / PI;

      // 满足你的要求：抬头 Pitch 增加 (翻转 Pitch 的正负方向)
      abs_pitch = -abs_pitch; 

      // 记录 DMP 首次输出数据的时间
      if (dmp_start_time == 0) dmp_start_time = millis();

      // 4. 等待 DMP 完全收敛 (8 秒) 后捕捉绝对零点
      if (!is_zero_recorded) {
        if (millis() - dmp_start_time < 8000) { 
          static uint32_t last_dot_time = 0;
          if (millis() - last_dot_time > 1000) {
             Serial1.print("Warming up DMP... ");
             Serial1.print(8 - (millis() - dmp_start_time)/1000);
             Serial1.println(" seconds left");
             last_dot_time = millis();
          }
          return; 
        } else {
          initial_pitch = abs_pitch;
          initial_roll  = abs_roll;
          initial_yaw   = abs_yaw;
          is_zero_recorded = true;
          Serial1.println("\r\n[DMP Converged! Zero Point Recorded!]");
        }
      }

      // 5. 零点捕捉完毕，输出干净的相对角度
      if (is_zero_recorded) {
        // ========== 修复了之前的乱码截断部分 ==========
        double pitch = abs_pitch - initial_pitch;
        double roll  = abs_roll  - initial_roll;
        double yaw   = abs_yaw   - initial_yaw;

        // 防止转圈时角度溢出（规范在 ±180 度以内）
        if (yaw > 180.0) yaw -= 360.0;   else if (yaw < -180.0) yaw += 360.0;
        if (pitch > 180.0) pitch -= 360.0; else if (pitch < -180.0) pitch += 360.0;
        if (roll > 180.0) roll -= 360.0;   else if (roll < -180.0) roll += 360.0;

        // 串口直观调试输出
        Serial1.print("Pitch:"); Serial1.print(pitch);
        Serial1.print("\tRoll:"); Serial1.print(roll);
        Serial1.print("\tYaw:"); Serial1.println(yaw);

      
      
            // ================= 数据打包发送 =================
      int16_t send_pitch = (int16_t)(pitch * 100);
      int16_t send_yaw   = (int16_t)(yaw * 100);

      // ★ 读取开火按键状态：因为是下拉，按下连通 3.3V 时为 HIGH (1)
      uint8_t fire_command = (digitalRead(PB12) == HIGH) ? 1 : 0;

      rece_buf[0] = 5; // ★ 数据长度变为 5
      rece_buf[1] = (send_pitch >> 8) & 0xFF; 
      rece_buf[2] = send_pitch & 0xFF;        
      rece_buf[3] = (send_yaw >> 8) & 0xFF;   
      rece_buf[4] = send_yaw & 0xFF;          
      rece_buf[5] = fire_command; // ★ 第5个字节装填开火指令

      SEND_BUF(rece_buf); 
       
        // ==========================================
      }
    }
  }
}