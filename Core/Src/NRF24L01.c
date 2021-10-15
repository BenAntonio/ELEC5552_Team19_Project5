 /**
  ******************************************************************************
  * File           : NRF24L01.c
  * Brief          : STM32 Development Board RF Library
  * Authors		   : Team 19
  * Version        : 1.0
  * Created		   : Sept 29, 2021
  * Last Modified  : Oct 15, 2021
  ******************************************************************************
  */

/*
 * NRF24L01.c
 *
 *  Created on: Sep 29, 2021
 *      Author: Team 19
 */

#include "stm32f4xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF_SPI &hspi1


#define NRF24_CE_PORT	GPIOB
#define NRF24_CE_PIN	GPIO_PIN_10 // Chip Enable Pin

#define NRF24_CSN_PORT	GPIOE
#define NRF24_CSN_PIN	GPIO_PIN_15 // Chip Select Pin

void CSselect (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

void CSunselect (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}

void CEenable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CEdisable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

void nrf24WrReg (uint8_t Reg, uint8_t Data) // Write a data byte to the register
{
	uint8_t buf[2];
	buf[0] = Reg| (1<<5); // Write a 1 in the 5th position of the buffer, as this is required when sending a register address
	buf[1] = Data; // Data byte

	// Pull the CS Pin to low to select a device
	CSselect();

	HAL_SPI_Transmit(NRF_SPI, buf, 2, 1000); // Transmit message to SPI

	// Pull the CS Pin to HIGH to release the device
	CSunselect();
}

void nrf24WrRegMulti (uint8_t Reg, uint8_t *data, int size) // Write multiple data bytes at once
{
	uint8_t buf[2];
	buf[0] = Reg| (1<<5); // Write a 1 in the 5th position of the buffer, as this is required when sending a register address
	// buf[1] = data; // Data byte

	// Pull the CS Pin to LOW to select a device
	CSselect();

	HAL_SPI_Transmit(NRF_SPI, buf, 1, 100); // Transmit message to SPI
	HAL_SPI_Transmit(NRF_SPI, data, size, 1000);

	// Pull the CS Pin to HIGH to release the device
	CSunselect();
}

uint8_t nrf24RdReg (uint8_t Reg) // Read a data byte from the register
{
	uint8_t data=0;

	// Pull the CS Pin to LOW to select a device
	CSselect();

	HAL_SPI_Transmit(NRF_SPI, &Reg, 1, 100); // Send register address to where the data is read from
	HAL_SPI_Receive(NRF_SPI, &data, 1, 100); // Read 1 byte of data

	// Pull the CS Pin to HIGH to release the device
	CSunselect();

	return data;
}

void nrf24RdRegMulti (uint8_t Reg, uint8_t *data, int size) // Read multiple data bytes from the register
{

	// Pull the CS Pin to LOW to select a device
	CSselect();

	HAL_SPI_Transmit(NRF_SPI, &Reg, 1, 100); // Send register address to where the data is read from
	HAL_SPI_Receive(NRF_SPI, data, size, 1000); // Read "size" bytes of data

	// Pull the CS Pin to HIGH to release the device
	CSunselect();

}


void nrfsendCmd (uint8_t command) //send standalone commands that don't need a register
{
	CSselect(); //select the device

	HAL_SPI_Transmit(NRF_SPI, &command, 1, 100);

	CSunselect(); //release the device
}


void nrfInit (void) // Send command to the NRF
{
	// disable chip before device configuration
	CEdisable();

	nrf24WrReg(CONFIG, 0); // Write 0 to config address to be configured later
	nrf24WrReg(EN_AA, 0); // Disable auto acknowledgement
	nrf24WrReg(EN_RXADDR, 0); // Disable data pipe
	nrf24WrReg(SETUP_AW, 0x03); // RX/TX address should have 5 bytes
	nrf24WrReg(SETUP_RETR, 0); // Disable retransmission
	nrf24WrReg(RF_CH, 0); // Write 0 as this is setup during send or receive
	nrf24WrReg(RF_SETUP, 0x0E); // Power is 0db, data rate is 2mbps

	// Enable chip after configuration
	CEenable();
}

// Function for configuring Tx mode
void nrfTxMode (uint8_t *Address, uint8_t channel)
{
	CEdisable(); // disable chip for config

	nrf24WrReg(RF_CH, channel); //channel select

	nrf24WrRegMulti(TX_ADDR, Address, 5); //write address to TX_ADDR

	// turn on the device
	uint8_t config = nrf24RdReg(CONFIG);
	config  = config | (1<<1);
	nrf24WrReg(CONFIG, config);

	CEenable(); //re-enable the chip

}


uint8_t nrfTransmit (uint8_t *data) //used to send the data
{
	uint8_t commandsend = 0; //command to send

	CSselect(); //select device

	commandsend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF_SPI, &commandsend, 1, 100); //transmit payload command


	HAL_SPI_Transmit(NRF_SPI, data, 32, 1000); //send "data width" bits of data

	CSunselect(); //release device

	HAL_Delay(1);

	uint8_t FIFOstatus = nrf24RdReg(FIFO_STATUS);

	if ((FIFOstatus&(1<<4)) && (!(FIFOstatus&(1<<3)))) //check that data is in FIFO
	{
		commandsend = FLUSH_TX;
		nrfsendCmd(commandsend);

		return 1;
	}

	return 0;
}

void nrfRxMode (uint8_t *Address, uint8_t channel)
{

	CEdisable();// disable the chip for config

	nrf24WrReg(RF_CH, channel);  // select the channel

	/*  NOTE: RF data pipe 1 is selected by default.
	 *  If other pipes are required, edit RX_ADDR_PX values accordingly.
	 */
	uint8_t EnRxAddr = nrf24RdReg(EN_RXADDR);
	EnRxAddr = EnRxAddr | (1<<1);
	nrf24WrReg(EN_RXADDR, EnRxAddr);

	nrf24WrRegMulti(RX_ADDR_P1, Address, 5);  // Write the pipe address

	nrf24WrReg (RX_PW_P1, 32);   // 32 bit payload size for pipe 1


	// power up the device in Rx mode
	uint8_t config = nrf24RdReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24WrReg (CONFIG, config);


	CEenable(); //re-enable the chip
}

uint8_t dataAvailability (int pipeNumber)
{
	uint8_t Status = nrf24RdReg(STATUS); // Read status register

	if ((Status&(1<<6))&&(Status&(pipeNumber<<1))) // Check to see if Rx data is ready
	{

		nrf24WrReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

void nrfReceive (uint8_t *data)
{
	uint8_t commandsend = 0;

	// select the device
	CSselect();

	// payload command
	commandsend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF_SPI, &commandsend, 1, 100);

	// Receive the payload
	HAL_SPI_Receive(NRF_SPI, data, 32, 1000);

	// Unselect the device
	CSunselect();

	HAL_Delay(1);

	commandsend = FLUSH_RX;
	nrfsendCmd(commandsend);
}

