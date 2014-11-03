#ifndef _RF24_CONFIG_H
#define _RF24_CONFIG_H

#define SPI_PORT            SPI1
#define SPI_SCK_PIN         GPIO_Pin_5     // PA5
#define SPI_MISO_PIN        GPIO_Pin_6     // PA6
#define SPI_MOSI_PIN        GPIO_Pin_7     // PA7
#define SPI_CS_PIN          GPIO_Pin_4     // PA4
#define SPI_GPIO_PORT       GPIOA
#define SPI_GPIO_ALT        GPIO_AF_5
// nRF24L01 CE (Chip Enable) pin
#define nRF24_CE_PORT       GPIOB
#define nRF24_CE_PIN        GPIO_Pin_11    // PB11

// nRF24L01 IRQ pin
#define nRF24_IRQ_PORT      GPIOB
#define nRF24_IRQ_PIN       GPIO_Pin_10    // PB10

#endif
