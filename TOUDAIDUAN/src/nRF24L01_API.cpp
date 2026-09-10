#include "nRF24L01_API.h"

/*********     24L01发送接收数据宽度定义   ***********/
#define TX_ADR_WIDTH    5     //5字节地址宽度
#define RX_ADR_WIDTH    5     //5字节地址宽度
#define TX_PLOAD_WIDTH  32    //32字节有效数据宽度
#define RX_PLOAD_WIDTH  32    //32字节有效数据宽度

const uchar TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //发送地址
const uchar RX_ADDRESS[RX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //接收地址

// =========== 引脚重映射到我们之前商定的 PA 口 ===========
#define CE_PIN   PA3
#define CSN_PIN  PA4
#define SCK_PIN  PA5
#define MOSI_PIN PA7
#define MISO_PIN PA6
#define IRQ_PIN  PA2

#define NRF_CE_HIGH()   digitalWrite(CE_PIN, HIGH)
#define NRF_CE_LOW()    digitalWrite(CE_PIN, LOW)
#define NRF_CSN_HIGH()  digitalWrite(CSN_PIN, HIGH)
#define NRF_CSN_LOW()   digitalWrite(CSN_PIN, LOW)
#define NRF_SCK_HIGH()  digitalWrite(SCK_PIN, HIGH)
#define NRF_SCK_LOW()   digitalWrite(SCK_PIN, LOW)
#define NRF_MOSI_HIGH() digitalWrite(MOSI_PIN, HIGH)
#define NRF_MOSI_LOW()  digitalWrite(MOSI_PIN, LOW)
#define NRF_MISO_READ() digitalRead(MISO_PIN)
#define NRF_IRQ_READ()  digitalRead(IRQ_PIN)

// 初始化NRF使用的引脚方向
void NRF24L01_Pin_Init(void) {
    pinMode(CE_PIN, OUTPUT);
    pinMode(CSN_PIN, OUTPUT);
    pinMode(SCK_PIN, OUTPUT);
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(MISO_PIN, INPUT);
    pinMode(IRQ_PIN, INPUT_PULLUP);
    
    NRF_CE_LOW();
    NRF_CSN_HIGH();
}

void delay_us(uchar num) {
    delayMicroseconds(num); // 直接使用Arduino自带的高精度微秒延时
}

void delay_150us(void) {
    delayMicroseconds(150);
}

uchar SPI_RW(uchar byte) {
    uchar bit_ctr;
    for(bit_ctr=0; bit_ctr<8; bit_ctr++) {
        if(byte & 0x80) NRF_MOSI_HIGH();
        else            NRF_MOSI_LOW();
        byte = (byte << 1); 
        NRF_SCK_HIGH();
        if(NRF_MISO_READ()) byte |= 0x01;
        NRF_SCK_LOW();
    }
    return byte;
}

uchar NRF24L01_Write_Reg(uchar reg,uchar value) {
    uchar status;
    NRF_CSN_LOW();                  
    status = SPI_RW(reg); 
    SPI_RW(value);
    NRF_CSN_HIGH();                  
    return status;
}

uchar NRF24L01_Read_Reg(uchar reg) {
    uchar value;
    NRF_CSN_LOW();              
    SPI_RW(reg); 
    value = SPI_RW(0xFF); // 发送空字节读取数据 (原文NOP可以替代为0xFF)
    NRF_CSN_HIGH();              
    return value;
}

uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len) {
    uchar status, u8_ctr;
    NRF_CSN_LOW();                           
    status = SPI_RW(reg);       
    for(u8_ctr=0; u8_ctr<len; u8_ctr++) {
        pBuf[u8_ctr] = SPI_RW(0XFF); 
    }
    NRF_CSN_HIGH();                  
    return status;         
}

uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len) {
    uchar status, u8_ctr;
    NRF_CSN_LOW();
    status = SPI_RW(reg); 
    for(u8_ctr=0; u8_ctr<len; u8_ctr++) {
        SPI_RW(*pBuf++); 
    }
    NRF_CSN_HIGH();
    return status;           
}       

uchar NRF24L01_RxPacket(uchar *rxbuf) {
    uchar state = NRF24L01_Read_Reg(STATUS);       
    NRF24L01_Write_Reg(nRF_WRITE_REG+STATUS, state); 
    if(state & RX_OK) {
        NRF_CE_LOW();
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        NRF24L01_Write_Reg(FLUSH_RX, 0xff); 
        NRF_CE_HIGH();
        delay_150us();
        return 0;
    }    
    return 1;
}

uchar NRF24L01_TxPacket(uchar *txbuf) {
    uchar state;
    NRF_CE_LOW(); 
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); 
    NRF_CE_HIGH(); 
    
    // 等待发送完成或者超时 (增加超时防止硬件故障导致死锁)
    uint32_t timeout = millis();
    while(NRF_IRQ_READ() == 1) {
        if(millis() - timeout > 100) return 0xff; // 100ms超时
    }
    
    state = NRF24L01_Read_Reg(STATUS);      
    NRF24L01_Write_Reg(nRF_WRITE_REG+STATUS, state); 
    
    if(state & MAX_TX) {
        NRF24L01_Write_Reg(FLUSH_TX, 0xff); 
        return MAX_TX;
    }
    if(state & TX_OK) {
        return TX_OK;
    }
    return 0xff; 
}
  
uchar NRF24L01_Check(void) {
    uchar check_in_buf[5] = {0x11,0x22,0x33,0x44,0x55};
    uchar check_out_buf[5] = {0x00};
    
    NRF_SCK_LOW();
    NRF_CSN_HIGH();    
    NRF_CE_LOW();
    
    NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR, check_in_buf, 5);
    NRF24L01_Read_Buf(nRF_READ_REG+TX_ADDR, check_out_buf, 5);
    
    if((check_out_buf[0] == 0x11) &&
       (check_out_buf[1] == 0x22) &&
       (check_out_buf[2] == 0x33) &&
       (check_out_buf[3] == 0x44) &&
       (check_out_buf[4] == 0x55)) {
        return 0;
    }
    return 1;
}

void NRF24L01_RT_Init(void) {
    NRF24L01_Pin_Init(); // 配置引脚模式
    
    NRF_CE_LOW();   
    NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0, RX_PLOAD_WIDTH);
    NRF24L01_Write_Reg(FLUSH_RX, 0xff); 
    NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR, (uchar*)TX_ADDRESS, TX_ADR_WIDTH);
    NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0, (uchar*)RX_ADDRESS, RX_ADR_WIDTH); 
    
    // 【关键修改：关闭自动应答和重发，实现极低延迟】
    NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA, 0x00);      // 0x00: 关闭自动应答
    NRF24L01_Write_Reg(nRF_WRITE_REG+SETUP_RETR, 0x00); // 0x00: 关闭自动重发
    
    NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR, 0x01); 
    NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH, 0);       
    NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP, 0x0F);  
    NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG, 0x0E);   // 0x0E: 发送模式
    NRF_CE_HIGH();   
}

void SEND_BUF(uchar *buf) {
    NRF_CE_LOW();
    NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG, 0x0E); // 确保在发送模式
    NRF_CE_HIGH();
    delayMicroseconds(15);
    NRF24L01_TxPacket(buf);
}