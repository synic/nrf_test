#include "stm32f30x.h"

#define SPI_PORT      SPI1
#define SPI_SCK_PIN   GPIO_Pin_5     // PA5
#define SPI_MISO_PIN  GPIO_Pin_6     // PA6
#define SPI_MOSI_PIN  GPIO_Pin_7     // PA7
#define SPI_CS_PIN    GPIO_Pin_4     // PA4
#define SPI_GPIO_PORT GPIOA
// nRF24L01 CE (Chip Enable) pin
#define nRF24_CE_PORT     GPIOB
#define nRF24_CE_PIN      GPIO_Pin_11    // PB11

// nRF24L01 IRQ pin
#define nRF24_IRQ_PORT    GPIOB
#define nRF24_IRQ_PIN     GPIO_Pin_10    // PB10

// Chip Enable Activates RX or TX mode
#define CE_L() GPIO_ResetBits(nRF24_CE_PORT,nRF24_CE_PIN)
#define CE_H() GPIO_SetBits(nRF24_CE_PORT,nRF24_CE_PIN)

// SPI Chip Select
#define CSN_L() GPIO_ResetBits(SPI_GPIO_PORT,SPI_CS_PIN)
#define CSN_H() GPIO_SetBits(SPI_GPIO_PORT,SPI_CS_PIN)

#define nRF24_CMD_WREG             0x20  // W_REGISTER -> Write command and status registers
#define nRF24_REG_TX_ADDR          0x10  // Transmit address

void spi_init(uint16_t prescaler) {
    SPI_InitTypeDef SPI;
    SPI.SPI_Mode = SPI_Mode_Master;
    SPI.SPI_BaudRatePrescaler = prescaler;
    SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI.SPI_CPOL = SPI_CPOL_Low;
    SPI.SPI_CPHA = SPI_CPHA_1Edge;
    SPI.SPI_CRCPolynomial = 7;
    SPI.SPI_DataSize = SPI_DataSize_8b;
    SPI.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI_PORT,&SPI);

    SPI_NSSInternalSoftwareConfig(SPI_PORT, SPI_NSSInternalSoft_Set);
}

void nrf24_init(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);   

    GPIO_InitTypeDef PORT;
    // Configure SPI pins
    PORT.GPIO_Speed = GPIO_Speed_2MHz;
    PORT.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
    PORT.GPIO_Mode = GPIO_Mode_AF;
    PORT.GPIO_OType = GPIO_OType_PP;
    PORT.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(SPI_GPIO_PORT, &PORT);

    PORT.GPIO_Speed = GPIO_Speed_2MHz;
    PORT.GPIO_Pin = SPI_MISO_PIN;
    PORT.GPIO_Mode = GPIO_Mode_AF;
    PORT.GPIO_OType = GPIO_OType_PP;
    PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI_GPIO_PORT, &PORT);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);

    // Configure CS pin as output with Push-Pull
    PORT.GPIO_Pin = SPI_CS_PIN;
    PORT.GPIO_Mode = GPIO_Mode_OUT;
    PORT.GPIO_OType = GPIO_OType_PP;
    PORT.GPIO_Speed = GPIO_Speed_2MHz;
    PORT.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPI_GPIO_PORT, &PORT);

    // Configure CE pin as output with Push-Pull
    PORT.GPIO_Pin = nRF24_CE_PIN;
    PORT.GPIO_Mode = GPIO_Mode_OUT;
    PORT.GPIO_Speed = GPIO_Speed_2MHz;
    PORT.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(nRF24_CE_PORT, &PORT);

    // Configure IRQ pin as input with Pull-Up
    PORT.GPIO_Pin = nRF24_IRQ_PIN;
    PORT.GPIO_Mode = GPIO_Mode_IN;
    PORT.GPIO_OType = GPIO_OType_PP;
    PORT.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(nRF24_IRQ_PORT, &PORT);

    spi_init(SPI_BaudRatePrescaler_4); // Which SPI speed do we need?
    SPI_Cmd(SPI_PORT,ENABLE);

    CSN_H();
    CE_L();
}

uint8_t nrf24_read_write(uint8_t data) {
    while (SPI_I2S_GetFlagStatus(SPI_PORT, SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
    SPI_I2S_SendData16(SPI_PORT, data); // Send byte to SPI
    while (SPI_I2S_GetFlagStatus(SPI_PORT, SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
    return SPI_I2S_ReceiveData16(SPI_PORT); // Read byte from SPI bus
}

uint8_t nrf24_read_buf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    uint8_t status,i;

    CSN_L();
    status = nrf24_read_write(reg);
    for (i = 0; i < count; i++) pBuf[i] = nrf24_read_write(0);
    CSN_L();

    return status;
}


uint8_t nrf24_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    uint8_t status,i;

    CSN_L();
    status = nrf24_read_write(reg);
    for (i = 0; i < count; i++) nrf24_read_write(*pBuf++);
    CSN_H();

    return status;
}


uint8_t nrf24_check(void) {
    uint8_t txbuf[5] = { 0xA8,0xA8,0xA8,0xA8,0xA8 };
    uint8_t rxbuf[5];
    uint8_t i;

    nrf24_write_buf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR, txbuf, 5); // Write fake TX address
    nrf24_read_buf(nRF24_REG_TX_ADDR, rxbuf, 5); // Try to read TX_ADDR register

    for (i = 0; i < 5; i++) if (rxbuf[i] != txbuf[i]) return 1;

    return 0;
}



int main(void) {
    if(SysTick_Config(SystemCoreClock / 1000)) {
        while(1) { }
    }

    nrf24_init();

    if (nrf24_check() != 0) {
        while(1); // halted nrf24 check
    }

    while(1) {

    }

    return 0;
}

