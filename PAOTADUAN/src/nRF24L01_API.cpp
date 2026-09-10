#include "nRF24L01_API.h"

#define TX_ADR_WIDTH    5    
#define RX_ADR_WIDTH    5    
#define TX_PLOAD_WIDTH  32   
#define RX_PLOAD_WIDTH  32   

const uchar TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; 
const uchar RX_ADDRESS[RX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; 

// =========== F405RG 引脚映射 ===========
#define CE_PIN   PC4
#define CSN_PIN  PC5
#define SCK_PIN  PB13
#define MISO_PIN PB14
#define MOSI_PIN PB15
#define IRQ_PIN  PC6

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
    value = SPI_RW(0xFF); 
    NRF_CSN_HIGH();              
    return value;
}

uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len) {
    uchar status, u8_ctr;
    NRF_CSN_LOW();                           
    status = SPI_RW(reg);       
    for(u8_ctr=0; u8_ctr<len; u8_ctr++) pBuf[u8_ctr] = SPI_RW(0XFF); 
    NRF_CSN_HIGH();                  
    return status;         
}

uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len) {
    uchar status, u8_ctr;
    NRF_CSN_LOW();
    status = SPI_RW(reg); 
    for(u8_ctr=0; u8_ctr<len; u8_ctr++) SPI_RW(*pBuf++); 
    NRF_CSN_HIGH();
    return status;           
}       

uchar NRF24L01_RxPacket(uchar *rxbuf) {
    uchar state = NRF24L01_Read_Reg(STATUS);       
    NRF24L01_Write_Reg(nRF_WRITE_REG+STATUS, state); // 清除中断标志
    if(state & RX_OK) { // 如果接收到数据
        NRF_CE_LOW(); // 暂停接收以读取数据
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        NRF24L01_Write_Reg(FLUSH_RX, 0xff); 
        NRF_CE_HIGH(); // 恢复接收
        return 0; // 成功
    }    
    return 1; // 无数据
}

uchar NRF24L01_Check(void) {
    uchar check_in_buf[5] = {0x11,0x22,0x33,0x44,0x55};
    uchar check_out_buf[5] = {0x00};
    NRF_SCK_LOW();
    NRF_CSN_HIGH();    
    NRF_CE_LOW();
    NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR, check_in_buf, 5);
    NRF24L01_Read_Buf(nRF_READ_REG+TX_ADDR, check_out_buf, 5);
    if(check_out_buf[0] == 0x11 && check_out_buf[1] == 0x22 && check_out_buf[2] == 0x33 && check_out_buf[3] == 0x44 && check_out_buf[4] == 0x55) return 0;
    return 1;
}

// 接收端专用初始化
void NRF24L01_RX_Init(void) {
    NRF24L01_Pin_Init(); 
    NRF_CE_LOW();   
    NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0, RX_PLOAD_WIDTH);
    NRF24L01_Write_Reg(FLUSH_RX, 0xff); 
    NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR, (uchar*)TX_ADDRESS, TX_ADR_WIDTH);
    NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0, (uchar*)RX_ADDRESS, RX_ADR_WIDTH); 
    
    // 关闭自动应答以实现极低延迟
    NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA, 0x00);      
    NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR, 0x01); 
    NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH, 0);       
    NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP, 0x0F);  
    
    // 0x0F = PWR_UP(上电) + CRCO(2字节校验) + PRX(接收模式)
    NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG, 0x0F);   
    
    NRF_CE_HIGH(); // CE拉高，模块开始持续监听空气中的信号 
}