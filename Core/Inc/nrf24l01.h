#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "main.h"

#define NRF_TX_TIMEOUT      2000
#define NRF_RX_TIMEOUT      2000
#define DEFAULT_CHANNEL     100

#define CONFIG              0x00
#define EN_AA               0x01
#define EN_RXADDR           0x02
#define SETUP_AW            0x03
#define SETUP_RETR          0x04
#define RF_CH               0x05
#define RF_SETUP            0x06
#define STATUS              0x07
#define OBSERVE_TX          0x08
#define RPD                 0x09
#define RX_ADDR_P0          0x0A
#define RX_ADDR_P1          0x0B
#define RX_ADDR_P2          0x0C
#define RX_ADDR_P3          0x0D
#define RX_ADDR_P4          0x0E
#define RX_ADDR_P5          0x0F
#define TX_ADDR             0x10
#define RX_PW_P0            0x11
#define RX_PW_P1            0x12
#define RX_PW_P2            0x13
#define RX_PW_P3            0x14
#define RX_PW_P4            0x15
#define RX_PW_P5            0x16
#define FIFO_STATUS         0x17
#define DYNPD               0x1C
#define FEATURE             0x1D

#define NRF_OK              200
#define NRF_NOT_FOUND       404
#define NRF_LARGE_PAYLOAD   413
#define NRF_UNAVAILABLE     503

#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define REGISTER_MASK       0x1F
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define W_ACK_PAYLOAD       0xA8
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define NOP                 0xFF

typedef enum {
	VERY_LOW,
	LOW,
	MID,
	HIGH
}p_level;

typedef enum {
    _250KBS,
	_1MBS,
	_2MBS
}dat_rate;

typedef enum {
    PWR_DOWN,
	STANDBY,
	TX_MODE,
	RX_MODE
}mode_nrf;

typedef enum {
    NO_CRC,
	CRC_8,
	CRC_16
}crc_len;

typedef struct {
	GPIO_TypeDef *CSN_port;
	uint16_t CSN_pin;
	GPIO_TypeDef *CE_port;
	uint16_t CE_pin;
	GPIO_TypeDef *IRQ_port;
	uint16_t IRQ_pin;
	SPI_HandleTypeDef *hSPIx;
	p_level pa;
	dat_rate bitRate;
	mode_nrf mode;
	crc_len crc;
}nrf24;

void NRF24_Init(nrf24 *node);
void NRF24_Set_DataRate(nrf24 *node, dat_rate _bitRate);
void NRF24_Set_PALevel(nrf24 *node, p_level pwr);
void NRF24_Set_Channel(nrf24 *node, uint8_t channel);
void NRF24_Set_Mode(nrf24 *node, mode_nrf _mode);
void NRF24_Set_CrcLentgh(nrf24 *node, crc_len _len);
void NRF24_Set_TxAddress(nrf24 *node, uint8_t *Address);
uint8_t NRF24_Transmit(nrf24 *node, uint8_t *data, uint8_t len);
void NRF24_Set_RxPipe(nrf24 *node, uint8_t *addr, uint8_t pipe, uint8_t payload);
uint8_t NRF24_Available(nrf24 *node, uint8_t pipenum);
void NRF24_Receive(nrf24 *node, uint8_t *data, uint8_t len);
void NRF24_ReadAll(nrf24 *node, uint8_t *data);

#endif
