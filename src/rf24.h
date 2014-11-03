#ifndef _RF24_H
#define _RF24_H

#include "stm32f30x.h"
#include "rf24_config.h"
#include "aolib.h"

// Chip Enable Activates RX or TX mode
#define CE_L() GPIO_ResetBits(nRF24_CE_PORT, nRF24_CE_PIN)
#define CE_H() GPIO_SetBits(nRF24_CE_PORT, nRF24_CE_PIN)

// SPI Chip Select
#define CSN_L() GPIO_ResetBits(SPI_GPIO_PORT, SPI_CS_PIN)
#define CSN_H() GPIO_SetBits(SPI_GPIO_PORT, SPI_CS_PIN)

#define _BV(x) (1<<(x))

typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW, 
    RF24_PA_HIGH, 
    RF24_PA_MAX, 
    RF24_PA_ERROR 
} rf24_pa_dbm_e;

typedef enum {
    RF24_1MBPS = 0, 
    RF24_2MBPS, 
    RF24_250KBPS 
} rf24_datarate_e;

typedef enum {
    RF24_CRC_DISABLED = 0, 
    RF24_CRC_8, 
    RF24_CRC_16 
} rf24_crclength_e;

class RF24 {
    private:
        bool wide_band;
        bool p_variant;
        uint8_t payload_size;
        bool ack_payload_available;
        bool dynamic_payloads_enabled;
        uint8_t ack_payload_length;
        uint64_t pipe0_reading_address;
    
    protected:
        uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);
        uint8_t read_register(uint8_t reg);

        uint8_t write_register(uint8_t reg, const uint8_t* buf, uint8_t len);
        uint8_t write_register(uint8_t reg, uint8_t value);

        uint8_t write_payload(const void* buf, uint8_t len);
        uint8_t read_payload(void* buf, uint8_t len);

        uint8_t flush_rx(void);
        uint8_t flush_tx(void);

        // status functions
        uint8_t get_status(void);

        void toggle_features(void);

        // spi stuff
        uint8_t spi_transfer(uint8_t data);
        void spi_init(uint16_t prescaler);
        void nrf24_init(void);

    public:
        RF24();
        void begin(void);
        void startListening(void);
        void stopListening(void);
        bool write(const void* buf, uint8_t len);
        bool available(void);
        bool read(void* buf, uint8_t len);
        void openWritingPipe(uint64_t address);
        void openReadingPipe(uint8_t number, uint64_t address);
        void setRetries(uint8_t delay, uint8_t count);
        void setChannel(uint8_t channel);
        void setPayloadSize(uint8_t size);
        uint8_t getPayloadSize(void);
        uint8_t getDynamicPayloadSize(void);
        void enableAckPayload(void);
        void enableDynamicPayloads(void);
        bool isPVariant(void);
        void setAutoAck(bool enable);
        void setAutoAck(uint8_t pipe, bool enable);
        void setPALevel(rf24_pa_dbm_e level);
        rf24_pa_dbm_e getPALevel(void);
        bool setDataRate(rf24_datarate_e speed);
        rf24_datarate_e getDataRate(void);
        void setCRCLength(rf24_crclength_e length);
        rf24_crclength_e getCRCLength(void);
        void disableCRC(void);
        void printDetails(void);
        void powerDown(void);
        void powerUp(void);
        bool available(uint8_t* pipe_num);
        void startWrite(const void* buf, uint8_t len);
        void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
        bool isAckPayloadAvailable(void);
        void whatHappened(bool& tx_ok, bool& tx_fail, bool& rx_ready);
        bool testCarrier(void);
        bool testRPD(void);
        bool isValid(void) { return true; }
};
        
#endif
