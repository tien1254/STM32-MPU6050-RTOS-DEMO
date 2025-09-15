#include "nrf24l01.h"

static void NRF24_CE_Enable(nrf24 *node);
static void NRF24_CE_Disable(nrf24 *node);
static void NRF24_CSN_Enable(nrf24 *node);
static void NRF24_CSN_Disable(nrf24 *node);
static void NRF24_Write_Reg(nrf24 *node, uint8_t reg, uint8_t data);
static void NRF24_Write_Buffer(nrf24 *node, uint8_t reg, uint8_t *data, uint16_t len);
static uint8_t NRF24_Read_Reg(nrf24 *node, uint8_t reg);
static void NRF24_Read_Buffer(nrf24 *node, uint8_t reg, uint8_t *data, uint16_t len);
static void NRF24_Send_Cmd(nrf24 *node, uint8_t cmd);
static void NRF24_Reset(nrf24 *node, uint8_t reg);

void NRF24_Init(nrf24 *node)
{
	NRF24_CE_Disable(node);
	NRF24_Write_Reg(node, CONFIG, 0);
	node->crc = NO_CRC;
	NRF24_Write_Reg(node, EN_AA, 0x00);
	NRF24_Write_Reg(node, EN_RXADDR, 0x00);
	NRF24_Write_Reg(node, SETUP_AW, 0x03);
	NRF24_Write_Reg(node, SETUP_RETR, 0);
	NRF24_Set_DataRate(node, _250KBS);
	NRF24_Set_PALevel(node, HIGH);
	NRF24_Set_Channel(node, DEFAULT_CHANNEL);
	NRF24_Set_Mode(node, STANDBY);
	NRF24_CE_Enable(node);
}

void NRF24_Set_DataRate(nrf24 *node, dat_rate _bitRate)
{
	NRF24_CE_Disable(node);
	uint8_t config = NRF24_Read_Reg(node, RF_SETUP);

	switch (_bitRate)
	{
		case _250KBS:
			config |= (1 << 5);
			config &=~(1 << 3);
			node->bitRate = _250KBS;
			break;

		case _1MBS:
			config &=~(1 << 5);
			config &=~(1 << 3);
			node->bitRate = _1MBS;
			break;

		case _2MBS:
			config &=~(1 << 5);
			config |= (1 << 3);
			node->bitRate = _2MBS;
			break;

		default:
			break;
	}
	NRF24_Write_Reg(node, RF_SETUP, config);
	NRF24_CE_Enable(node);
}

void NRF24_Set_PALevel(nrf24 *node, p_level pwr)
{
	NRF24_CE_Disable(node);
	uint8_t config = NRF24_Read_Reg(node, RF_SETUP);

	switch (pwr) {
		case VERY_LOW:
			config &=~(1 << 2);
			config &=~(1 << 1);
			node->pa = VERY_LOW;
			break;

		case LOW:
			config &=~(1 << 2);
			config |= (1 << 1);
			node->pa = LOW;
			break;

		case MID:
			config |= (1 << 2);
			config &=~(1 << 1);
			node->pa = MID;
			break;

		case HIGH:
			config |= (1 << 2) | (1 << 1);
			node->pa = HIGH;
			break;

		default:
			break;
	}
	NRF24_Write_Reg(node, RF_SETUP, config);
	NRF24_CE_Enable(node);
}

void NRF24_Set_Channel(nrf24 *node, uint8_t channel)
{
	NRF24_CE_Disable(node);
	NRF24_Write_Reg(node, RF_CH, channel);
	NRF24_CE_Enable(node);
}

void NRF24_Set_Mode(nrf24 *node, mode_nrf _mode)
{
	uint8_t config = NRF24_Read_Reg(node, CONFIG);

	switch (_mode)
	{
		case PWR_DOWN:
			config &=~(1 << 1);
			node->mode = PWR_DOWN;
			break;

		case STANDBY:
			NRF24_CE_Disable(node);
			config |= (1 << 1);
			node->mode = STANDBY;
			break;

		case TX_MODE:
			NRF24_CE_Enable(node);
			config |= (1 << 1);
			config &=~(1 << 0);
			node->mode = TX_MODE;
			break;

		case RX_MODE:
			NRF24_CE_Enable(node);
			config |= (1 << 1) | (1 << 0);
			node->mode = RX_MODE;
			break;

		default:
			break;
	}
	NRF24_Write_Reg(node, CONFIG, config);
}

void NRF24_Set_CrcLentgh(nrf24 *node, crc_len _len)
{
	NRF24_CE_Disable(node);
	uint8_t config = NRF24_Read_Reg(node, CONFIG);

	switch (_len)
	{
		case NO_CRC:
			config &=~(1 << 3);
			config &=~(1 << 2);
			NRF24_Write_Reg(node, CONFIG, config);
			node->crc = NO_CRC;
			break;

		case CRC_8:
			config |= (1 << 3);
			config &=~(1 << 2);
			node->crc = CRC_8;
			break;

		case CRC_16:
			config |= (1 << 3) | (1 << 2);
			node->crc = CRC_16;
			break;

		default:
			break;
	}
	NRF24_Write_Reg(node, CONFIG, config);
	NRF24_CE_Enable(node);
}

void NRF24_Set_TxAddress(nrf24 *node, uint8_t *Address)
{
	NRF24_CE_Disable(node);
	NRF24_Write_Buffer(node, TX_ADDR, Address, 5);
	NRF24_CE_Enable(node);
}

uint8_t NRF24_Transmit(nrf24 *node, uint8_t *data, uint8_t len)
{
	uint8_t cmdtosend = 0;

	NRF24_CSN_Enable(node);
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(node->hSPIx, &cmdtosend, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);

	if(len < 33) HAL_SPI_Transmit(node->hSPIx, data, len, NRF_TX_TIMEOUT);
	NRF24_CSN_Disable(node);

	uint8_t timeout = 0;
	while(timeout++ < 10)
	{
		for(int i=0; i<200; ++i)
		{
			__ASM("NOP");
		}
		uint8_t fifostatus = NRF24_Read_Reg(node, FIFO_STATUS);

		if ((fifostatus & (1 << 4)) && (!(fifostatus & (1 << 3))))
		{
			cmdtosend = FLUSH_TX;
			NRF24_Send_Cmd(node, cmdtosend);
			NRF24_Reset(node, FIFO_STATUS);
			return (1);
		}
	}
	return (0);
}

void NRF24_Set_RxPipe(nrf24 *node, uint8_t *addr, uint8_t pipe, uint8_t payload)
{
	NRF24_CE_Disable(node);
	NRF24_Reset(node, STATUS);
	uint8_t en_rxaddr = NRF24_Read_Reg(node, EN_RXADDR);
	en_rxaddr |= (1 << pipe);
	NRF24_Write_Reg(node, EN_RXADDR, en_rxaddr);

	switch(pipe)
	{
		case 0:
			NRF24_Write_Buffer(node, RX_ADDR_P0, addr, 5);
			NRF24_Write_Reg(node, RX_PW_P0, payload);
			break;

		case 1:
			NRF24_Write_Buffer(node, RX_ADDR_P1, addr, 5);
			NRF24_Write_Reg(node, RX_PW_P1, payload);
			break;

		case 2:
			NRF24_Write_Reg(node, RX_ADDR_P2, addr[0]);
			NRF24_Write_Reg(node, RX_PW_P2, payload);
			break;

		case 3:
			NRF24_Write_Reg(node, RX_ADDR_P3, addr[0]);
			NRF24_Write_Reg(node, RX_PW_P3, payload);
			break;

		case 4:
			NRF24_Write_Reg(node, RX_ADDR_P4, addr[0]);
			NRF24_Write_Reg(node, RX_PW_P4, payload);
			break;

		case 5:
			NRF24_Write_Reg(node, RX_ADDR_P5, addr[0]);
			NRF24_Write_Reg(node, RX_PW_P5, payload);
			break;

		default:
			break;
	}
	NRF24_CE_Enable(node);
}

uint8_t NRF24_Available(nrf24 *node, uint8_t pipenum)
{
	uint8_t status = NRF24_Read_Reg(node, STATUS);
	if((status - 64 == 0) && pipenum == 0)
	{
		NRF24_Write_Reg(node, STATUS, (1 << 6));
		return 1;
	}
	else if((status & (1 << 6)) && (status & (pipenum << 1)))
	{
		NRF24_Write_Reg(node, STATUS, (1 << 6));
		return 1;
	}
	return 0;
}

void NRF24_Receive(nrf24 *node, uint8_t *data, uint8_t len)
{
	uint8_t cmdtosend = 0;

	NRF24_CSN_Enable(node);
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(node->hSPIx, &cmdtosend, 1, 100);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(node->hSPIx, data, len, NRF_RX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	NRF24_CSN_Disable(node);
	HAL_Delay(1);
	cmdtosend = FLUSH_RX;
	NRF24_Send_Cmd(node, cmdtosend);
}

void NRF24_ReadAll(nrf24 *node, uint8_t *data)
{
	for (int i = 0; i < 10; i++)
		*(data + i) = NRF24_Read_Reg(node, i);

	NRF24_Read_Buffer(node, RX_ADDR_P0, (data + 10), 5);
	NRF24_Read_Buffer(node, RX_ADDR_P1, (data + 15), 5);

	*(data + 20) = NRF24_Read_Reg(node, RX_ADDR_P2);
	*(data + 21) = NRF24_Read_Reg(node, RX_ADDR_P3);
	*(data + 22) = NRF24_Read_Reg(node, RX_ADDR_P4);
	*(data + 23) = NRF24_Read_Reg(node, RX_ADDR_P5);

	NRF24_Read_Buffer(node, RX_ADDR_P0, (data + 24), 5);

	for (int i = 29; i < 38; i++)
		*(data + i) = NRF24_Read_Reg(node, i - 12);
}

static void NRF24_CE_Enable(nrf24 *node)
{
	HAL_GPIO_WritePin(node->CE_port, node->CE_pin, GPIO_PIN_SET);
}

static void NRF24_CE_Disable(nrf24 *node)
{
	HAL_GPIO_WritePin(node->CE_port, node->CE_pin, GPIO_PIN_RESET);
}

static void NRF24_CSN_Enable(nrf24 *node)
{
	HAL_GPIO_WritePin(node->CSN_port, node->CSN_pin, GPIO_PIN_RESET);
}

static void NRF24_CSN_Disable(nrf24 *node)
{
	HAL_GPIO_WritePin(node->CSN_port, node->CSN_pin, GPIO_PIN_SET);
}

static void NRF24_Write_Reg(nrf24 *node, uint8_t reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = (reg | 1 << 5);
	buf[1] = data;
	NRF24_CSN_Enable(node);
	HAL_SPI_Transmit(node->hSPIx, buf, 2, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	NRF24_CSN_Disable(node);
}

static void NRF24_Write_Buffer(nrf24 *node, uint8_t reg, uint8_t *data, uint16_t len)
{
	uint8_t buf[2];
	buf[0] = (reg | 1 << 5);
	NRF24_CSN_Enable(node);
	HAL_SPI_Transmit(node->hSPIx, buf, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(node->hSPIx, data, len, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	NRF24_CSN_Disable(node);
}

static uint8_t NRF24_Read_Reg(nrf24 *node, uint8_t reg)
{
	uint8_t data = 0;
	NRF24_CSN_Enable(node);
	HAL_SPI_Transmit(node->hSPIx, &reg, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(node->hSPIx, &data, 1, NRF_RX_TIMEOUT);
	NRF24_CSN_Disable(node);
	return data;
}

static void NRF24_Read_Buffer(nrf24 *node, uint8_t reg, uint8_t *data, uint16_t len)
{
	NRF24_CSN_Enable(node);
	HAL_SPI_Transmit(node->hSPIx, &reg, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(node->hSPIx, data, len, NRF_RX_TIMEOUT);
	NRF24_CSN_Disable(node);
}

static void NRF24_Send_Cmd(nrf24 *node, uint8_t cmd)
{
	NRF24_CSN_Enable(node);
	HAL_SPI_Transmit(node->hSPIx, &cmd, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	NRF24_CSN_Disable(node);
}

static void NRF24_Reset(nrf24 *node, uint8_t reg)
{
	if(reg == STATUS)
	{
		NRF24_Write_Reg(node, STATUS, 0x00);
	}
	else if(reg == FIFO_STATUS)
	{
		NRF24_Write_Reg(node, FIFO_STATUS, 0x11);
	}
	else
	{
		NRF24_Write_Reg(node, CONFIG, 0x08);
		NRF24_Write_Reg(node, EN_AA, 0x3F);
		NRF24_Write_Reg(node, EN_RXADDR, 0x03);
		NRF24_Write_Reg(node, SETUP_AW, 0x03);
		NRF24_Write_Reg(node, SETUP_RETR, 0x03);
		NRF24_Write_Reg(node, RF_CH, 0x02);
		NRF24_Write_Reg(node, RF_SETUP, 0x0E);
		NRF24_Write_Reg(node, STATUS, 0x00);
		NRF24_Write_Reg(node, OBSERVE_TX, 0x00);
		NRF24_Write_Reg(node, RPD, 0x00);
		uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		NRF24_Write_Buffer(node, RX_ADDR_P0, rx_addr_p0_def, 5);
		uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
		NRF24_Write_Buffer(node, RX_ADDR_P1, rx_addr_p1_def, 5);
		NRF24_Write_Reg(node, RX_ADDR_P2, 0xC3);
		NRF24_Write_Reg(node, RX_ADDR_P3, 0xC4);
		NRF24_Write_Reg(node, RX_ADDR_P4, 0xC5);
		NRF24_Write_Reg(node, RX_ADDR_P5, 0xC6);
		uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		NRF24_Write_Buffer(node, TX_ADDR, tx_addr_def, 5);
		NRF24_Write_Reg(node, RX_PW_P0, 0);
		NRF24_Write_Reg(node, RX_PW_P1, 0);
		NRF24_Write_Reg(node, RX_PW_P2, 0);
		NRF24_Write_Reg(node, RX_PW_P3, 0);
		NRF24_Write_Reg(node, RX_PW_P4, 0);
		NRF24_Write_Reg(node, RX_PW_P5, 0);
		NRF24_Write_Reg(node, FIFO_STATUS, 0x11);
		NRF24_Write_Reg(node, DYNPD, 0);
		NRF24_Write_Reg(node, FEATURE, 0);
	}
}
