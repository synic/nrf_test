#include "nrf24l01.h"
#include "rf24.h"
#include "aolib.h"
#include "binary.h"
#include <cstddef>

#define PROGMEM
#define pgm_read_word(p) (*(p)) 
#define pgm_read_byte(p) (*(p)) 

void RF24::spi_init(uint16_t prescaler) {
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
    SPI_Init(SPI_PORT, &SPI);
    SPI_RxFIFOThresholdConfig(SPI_PORT, SPI_RxFIFOThreshold_QF);
    SPI_NSSInternalSoftwareConfig(SPI_PORT, SPI_NSSInternalSoft_Set);
}

void RF24::nrf24_init(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);   

    GPIO_InitTypeDef PORT;
    // Configure SPI pins
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    PORT.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
    PORT.GPIO_Mode = GPIO_Mode_AF;
    PORT.GPIO_OType = GPIO_OType_PP;
    PORT.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(SPI_GPIO_PORT, &PORT);

    PORT.GPIO_Speed = GPIO_Speed_50MHz;
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
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_GPIO_PORT, &PORT);

    // Configure CE pin as output with Push-Pull
    PORT.GPIO_Pin = nRF24_CE_PIN;
    PORT.GPIO_Mode = GPIO_Mode_OUT;
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    PORT.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(nRF24_CE_PORT, &PORT);

    // Configure IRQ pin as input with Pull-Up
    PORT.GPIO_Pin = nRF24_IRQ_PIN;
    PORT.GPIO_Mode = GPIO_Mode_IN;
    PORT.GPIO_OType = GPIO_OType_PP;
    PORT.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(nRF24_IRQ_PORT, &PORT);

    spi_init(SPI_BaudRatePrescaler_16); // Which SPI speed do we need?
    SPI_Cmd(SPI_PORT, ENABLE);
}

uint8_t RF24::spi_transfer(uint8_t data) {
    while((SPI_PORT->SR & SPI_I2S_FLAG_TXE) == RESET);
    SPI_SendData8(SPI_PORT, data);
    while((SPI_PORT->SR & SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_ReceiveData8(SPI_PORT);
}

uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len) {
    uint8_t status;

    CSN_L();
    status = spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    while(len--) {
        *buf++ = spi_transfer(0xff);
    }

    CSN_H();

    return status;
}

uint8_t RF24::read_register(uint8_t reg) {
    CSN_L();
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t result = spi_transfer(0xff);

    CSN_H();

    return result;
}

uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len) {
    uint8_t status;

    CSN_L();
    status = spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    while(len--) {
        spi_transfer(*buf++);
    }

    CSN_H();

    return status;
}

uint8_t RF24::write_register(uint8_t reg, uint8_t value) {
    uint8_t status;

    CSN_L();
    status = spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);
    CSN_H();

    return status;
}

uint8_t RF24::write_payload(const void* buf, uint8_t len) {
    uint8_t status;

    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

    uint8_t data_len = (len <= payload_size) ? len : payload_size;
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    CSN_L();
    status = spi_transfer(W_TX_PAYLOAD);
    while(data_len--) {
        spi_transfer(*current++);
    }
    while(blank_len--) {
        spi_transfer(0);
    }

    CSN_H();

    return status;
}

uint8_t RF24::read_payload(void* buf, uint8_t len) {
    uint8_t status;
    uint8_t* current = reinterpret_cast<uint8_t*>(buf);

    uint8_t data_len = (len <= payload_size) ? len : payload_size;
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    CSN_L();
    status = spi_transfer(R_RX_PAYLOAD);
    while(data_len--) {
        *current++ = spi_transfer(0xff);
    }

    while(blank_len--) {
        spi_transfer(0xff);
    }

    CSN_H();

    return status;
}

uint8_t RF24::flush_rx(void) {
    uint8_t status;

    CSN_L();
    status = spi_transfer(FLUSH_RX);
    CSN_H();

    return status;
}

uint8_t RF24::flush_tx(void) {
    uint8_t status;
    CSN_L();
    status = spi_transfer(FLUSH_TX);
    CSN_H();
    return status;
}

uint8_t RF24::get_status(void) {
    uint8_t status;

    CSN_L();
    status = spi_transfer(NOP);
    CSN_H();

    return status;
}

RF24::RF24() :
    wide_band(true), p_variant(false),
    payload_size(32), ack_payload_available(false), 
    dynamic_payloads_enabled(false),
    pipe0_reading_address(0)
{
}

void RF24::setChannel(uint8_t channel) {
    const uint8_t max_channel = 127;
    uint8_t ch = (channel <= max_channel) ? channel : max_channel;

    write_register(RF_CH, ch);
}

void RF24::setPayloadSize(uint8_t size) {
    const uint8_t max_payload_size = 32;
    payload_size = (size <= max_payload_size) ? size : max_payload_size;
}

uint8_t RF24::getPayloadSize(void) {
    return payload_size;
}

void RF24::begin(void) {
    nrf24_init();

    CE_L();
    CSN_H();

    delay_ms(5);

    write_register(SETUP_RETR, (B0100 << ARD) | (B1111 << ARC));
    setPALevel(RF24_PA_MAX);

    if(setDataRate(RF24_250KBPS)) {
        p_variant = true;
    }

    setDataRate(RF24_1MBPS);
    setCRCLength(RF24_CRC_16);
    write_register(DYNPD, 0);
    write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    setChannel(76);
    flush_rx();
    flush_tx();
}

void RF24::startListening(void) {
    write_register(CONFIG, read_register(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
    write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    if(pipe0_reading_address) {
        write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&pipe0_reading_address), 5);
    }

    flush_rx();
    flush_tx();

    CE_H();

    delay_us(130);
}

void RF24::stopListening(void) {
    CE_L();
    flush_tx();
    flush_rx();
}

void RF24::powerDown(void) {
    write_register(CONFIG, read_register(CONFIG) & ~_BV(PWR_UP));
}

void RF24::powerUp(void) {
    write_register(CONFIG, read_register(CONFIG) | _BV(PWR_UP));
}

bool RF24::write(const void* buf, uint8_t len) {
    bool result = false;
    startWrite(buf, len);

    uint8_t observe_tx;
    uint8_t status;
    uint32_t sent_at = millis();
    const uint32_t timeout = 500;
    do {
        status = read_register(OBSERVE_TX, &observe_tx, 1);
    }
    while(!(status & (_BV(TX_DS) | _BV(MAX_RT))) && (millis() - sent_at < timeout));

    bool tx_ok, tx_fail;
    whatHappened(tx_ok, tx_fail, ack_payload_available);

    result = tx_ok;
    if(ack_payload_available) {
        ack_payload_length = getDynamicPayloadSize();
    }

    powerDown();

    flush_tx();

    return result;
}

void RF24::startWrite(const void* buf, uint8_t len) {
    write_register(CONFIG, (read_register(CONFIG) | _BV(PWR_UP)) & ~_BV(PRIM_RX));
    delay_us(150);
    write_payload(buf, len);

    CE_H();
    delay_us(15);
    CE_L();
}

uint8_t RF24::getDynamicPayloadSize(void) {
    uint8_t result = 0;

    CSN_L();
    spi_transfer(R_RX_PL_WID);
    result = spi_transfer(0xff);
    CSN_H();

    return result;
}

bool RF24::available(void) {
    return available(NULL);
}

bool RF24::available(uint8_t* pipe_num) {
    uint8_t status = get_status();

    bool result = (status & _BV(RX_DR));

    if(result) {
        if(pipe_num) {
            *pipe_num = (status >> RX_P_NO) & B111;
        }

        write_register(STATUS, _BV(RX_DR));

        if(status & _BV(TX_DS)) {
            write_register(STATUS, _BV(TX_DS));
        }
    }

    return result;
}

bool RF24::read( void *buf, uint8_t len) {
    read_payload(buf, len);
    return read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

void RF24::whatHappened(bool& tx_ok, bool& tx_fail, bool& rx_ready) {
    uint8_t status = write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    tx_ok = status & _BV(TX_DS);
    tx_fail = status & _BV(MAX_RT);
    rx_ready = status & _BV(RX_DR);
}

void RF24::openWritingPipe(uint64_t value) {
    write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), 5);
    write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), 5);

    const uint8_t max_payload_size = 32;
    write_register(RX_PW_P0, (payload_size <= max_payload_size) ?  payload_size : max_payload_size);
}

static const uint8_t child_pipe[] PROGMEM = {
    RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};

static const uint8_t child_payload_size[] PROGMEM = {
    RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

static const uint8_t child_pipe_enable[] PROGMEM {
    ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void RF24::openReadingPipe(uint8_t child, uint64_t address) {
    if(child == 0) {
        pipe0_reading_address = address;
    }

    if(child <= 6) {
        if(child < 2) {
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 5);
        }
        else {
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);
        }

        write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

        write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
    }
}

void RF24::toggle_features(void) {
    CSN_L();
    spi_transfer(ACTIVATE);
    spi_transfer(0x73);
    CSN_H();
}

void RF24::enableDynamicPayloads(void) {
    write_register(FEATURE, read_register(FEATURE) | _BV(EN_DPL));

    if(!read_register(FEATURE)) {
        toggle_features();
        write_register(FEATURE, read_register(FEATURE) | _BV(EN_DPL));
    }

    write_register(DYNPD, read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

    dynamic_payloads_enabled = true;
}

void RF24::enableAckPayload(void) {
    write_register(FEATURE, read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));

    if(!read_register(FEATURE)) {
        toggle_features();
        write_register(FEATURE, read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));
    }

    write_register(DYNPD, read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

void RF24::writeAckPayload(uint8_t pipe, const void* buf, uint8_t len) {
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

    CSN_L();
    spi_transfer(W_ACK_PAYLOAD | (pipe & B111));
    const uint8_t max_payload_size = 32;
    uint8_t data_len = (len <= max_payload_size) ? len : max_payload_size;
    while(data_len--) {
        spi_transfer(*current++);
    }

    CSN_H();
}

bool RF24::isAckPayloadAvailable(void) {
    bool result = ack_payload_available;
    ack_payload_available = false;
    return result;
}

bool RF24::isPVariant(void) {
    return p_variant;
}

void RF24::setAutoAck(bool enable) {
    if(enable) {
        write_register(EN_AA, B111111);
    }
    else {
        write_register(EN_AA, 0);
    }
}

void RF24::setAutoAck(uint8_t pipe, bool enable) {
    if(pipe <= 6) {
        uint8_t en_aa = read_register(EN_AA);
        if(enable) {
            en_aa |= _BV(pipe);
        }
        else {
            en_aa &= ~_BV(pipe);
        }

        write_register(EN_AA, en_aa);
    }
}

bool RF24::testCarrier(void) {
    return (read_register(CD) & 1);
}

bool RF24::testRPD(void) {
    return (read_register(RPD) & 1);
}

void RF24::setPALevel(rf24_pa_dbm_e level) {
    uint8_t setup = read_register(RF_SETUP);
    setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

    // switch uses RAM (evil!)
    if(level == RF24_PA_MAX) {
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
    }
    else if(level == RF24_PA_HIGH) {
        setup |= _BV(RF_PWR_HIGH);
    }
    else if(level == RF24_PA_LOW) {
        setup |= _BV(RF_PWR_LOW);
    }
    else if(level == RF24_PA_MIN) {
        // nothing
    }
    else if(level == RF24_PA_ERROR) {
        // On error, go to maximum PA
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
    }

    write_register(RF_SETUP, setup);
}

rf24_pa_dbm_e RF24::getPALevel(void) {
    rf24_pa_dbm_e result = RF24_PA_ERROR;
    uint8_t power = read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

    if(power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) {
        result = RF24_PA_MAX;
    }
    else if(power == _BV(RF_PWR_HIGH)) {
        result = RF24_PA_HIGH;
    }
    else if(power == _BV(RF_PWR_LOW)) {
        result = RF24_PA_LOW;
    }
    else {
        result = RF24_PA_MIN;
    }

    return result;
}

bool RF24::setDataRate(rf24_datarate_e speed) {
    bool result = false;
    uint8_t setup = read_register(RF_SETUP);

    // HIGH and LOW '00' is 1Mbs - our default
    wide_band = false;
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
    if(speed == RF24_250KBPS) {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        wide_band = false ;
        setup |= _BV( RF_DR_LOW );
    }
    else {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if(speed == RF24_2MBPS) {
            wide_band = true ;
            setup |= _BV(RF_DR_HIGH);
        }
        else {
            // 1Mbs
            wide_band = false;
        }
    }
    write_register(RF_SETUP, setup);

    // Verify our result
    if(read_register(RF_SETUP) == setup) {
        result = true;
    }
    else {
        wide_band = false;
    }

    return result;
}

/****************************************************************************/

rf24_datarate_e RF24::getDataRate(void) {
    rf24_datarate_e result;
    uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
    
    // switch uses RAM (evil!)
    // Order matters in our case below
    if(dr == _BV(RF_DR_LOW)) {
        // '10' = 250KBPS
        result = RF24_250KBPS;
    }
    else if(dr == _BV(RF_DR_HIGH)) {
        // '01' = 2MBPS
        result = RF24_2MBPS;
    }
    else {
        // '00' = 1MBPS
        result = RF24_1MBPS;
    }

    return result;
}

/****************************************************************************/

void RF24::setCRCLength(rf24_crclength_e length) {
    uint8_t config = read_register(CONFIG) & ~(_BV(CRCO) | _BV(EN_CRC));
    
    // switch uses RAM (evil!)
    if(length == RF24_CRC_DISABLED) {
        // Do nothing, we turned it off above. 
    }
    else if(length == RF24_CRC_8) {
        config |= _BV(EN_CRC);
    }
    else {
        config |= _BV(EN_CRC);
        config |= _BV(CRCO);
    }

    write_register(CONFIG, config);
}

/****************************************************************************/

rf24_crclength_e RF24::getCRCLength(void) {
    rf24_crclength_e result = RF24_CRC_DISABLED;
    uint8_t config = read_register(CONFIG) & (_BV(CRCO) | _BV(EN_CRC));

    if(config & _BV(EN_CRC)) {
        if(config & _BV(CRCO)) {
            result = RF24_CRC_16;
        }
        else {
            result = RF24_CRC_8;
        }
    }

    return result;
}

/****************************************************************************/

void RF24::disableCRC(void) {
    uint8_t disable = read_register(CONFIG) & ~_BV(EN_CRC);
    write_register(CONFIG, disable);
}

/****************************************************************************/
void RF24::setRetries(uint8_t delay, uint8_t count) {
    write_register(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

