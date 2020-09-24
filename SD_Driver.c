

#include "SD_Driver.h"
#include "diskio.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_spi.h"
#include "stm32g0xx_hal_gpio.h"
#include "stm32g0xx_hal_tim.h"
#include <string.h>

#define STATUS uint8_t
#define VERSION uint8_t

#define SUCCESS 1
#define FAIL 0

#define SPI1_CS_PORT GPIOB
#define SPI1_CS_PIN GPIO_PIN_0

#define TIMEOUT 1000

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;

static volatile DSTATUS status = STA_NOINIT;
static volatile VERSION version = 0;



/*
 * Description: Asserts the CS signal for SPI1
 * Param: 	None
 * Return: 	None
 */
static void select()
{
	HAL_GPIO_WritePin(SPI1_CS_PORT, SPI1_CS_PIN, 0);
}

/*
 * Description: De-asserts the CS signal for SPI1
 * Param: 	None
 * Return: 	None
 */
static void deselect()
{
	HAL_GPIO_WritePin(SPI1_CS_PORT, SPI1_CS_PIN, 1);
}

/*
 * Description: Starts timer 6 from 0
 * Param: 	None
 * Return: 	None
 */
static void start_timer()
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	HAL_TIM_Base_Start(&htim6);
}

/*
 * Description: Returns the timer 6 count
 * Param: 	None
 * Return: 	Timer 6 count (uint16_t)
 */
static uint16_t get_timer_val()
{
	return __HAL_TIM_GET_COUNTER(&htim6);
}

/*
 * Description: Stops timer 6
 * Param: 	None
 * Return: 	None
 */
static void stop_timer()
{
	HAL_TIM_Base_Stop(&htim6);
}

/*
 * Description: Sends one byte while receiving one byte
 * Param: 	sendByte: Pointer to byte to be sent
 * Return: 	Received byte from exchange
 */
static uint8_t exchange_byte(uint8_t *sendByte)
{
	uint8_t receiveByte = 0xFF;

	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){};
	HAL_SPI_TransmitReceive(&hspi1, sendByte, &receiveByte, 1, TIMEOUT);

	return receiveByte;
}

/*
 * Description: Receives one byte while keeping MOSI high
 * Param: 	None
 * Return: 	Received byte
 */
static uint8_t receive_byte()
{
	uint8_t dummyData = 0xFF;
	return exchange_byte(&dummyData);
}

/*
 * Description: Wait for SD Card to be ready (SD Card keeps MISO high when ready)
 * Param: 	None
 * Return: 	0xFF if SD Card is ready, 0 if timed out
 */
static uint8_t wait_sd_ready()
{
	start_timer();
	uint8_t res = 0;

	while(res != 0xFF && get_timer_val() < 500)
	{
		res = receive_byte();
	}

	stop_timer();

	if(res == 0xFF)
		return res;
	else
		return 0;
}

/*
 * Description: Sends a command to the SD Card and returns the response
 * Param: 	cmd: Command to be sent
 * 			arg: Argument to be sent
 * 			responseTries: Number of times to try receive before giving up
 * 			ocr_buffer: pointer to buffer for extended response, if only R1
 * 						response is expected this can be NULL
 * Return: 	Returns the response from the SD Card
 */
static uint8_t send_command(uint8_t cmd, uint32_t arg, uint16_t responseTries, uint32_t *ocr_buffer)
{
	uint8_t res = 0xFF;
	uint8_t ocr[4];

	if(cmd == CMD0 || wait_sd_ready())		//Only send if ready
	{
		uint8_t arg_bytes[4];
		uint8_t crc = 0;

		/* SPLIT ARGS FOR SPI EXCHANGE */
		arg_bytes[0] = (uint8_t)(arg >> 24);
		arg_bytes[1] = (uint8_t)(arg >> 16);
		arg_bytes[2] = (uint8_t)(arg >> 8);
		arg_bytes[3] = (uint8_t)(arg >> 0);

		/* SET CORRECT CRC IF CMD0 OR CMD 8 */
		if(cmd == CMD0)
			crc = 0x95;
		if(cmd == CMD8)
			crc = 0x87;

		/* SEND COMMAND FRAME */
		exchange_byte(&cmd);

		for(int i = 0; i < 4; i++)
		{
			exchange_byte(&arg_bytes[i]);
		}

		exchange_byte(&crc);

		/* COLLECT RESPONSE */
		uint8_t tries = responseTries;

		while(tries && (res & 0x80))
		{
			tries--;
			res = receive_byte();
		}

		if(cmd == CMD0)
		{
			uint16_t count = 1000;
			while(count && res != 0x01)
			{
				count--;
				res = receive_byte();
			}
		}

		/* COLLECT OCR RESPONSE IF CMD8 OR CMD 58 */
		if(cmd == CMD8 || cmd == CMD58)
		{
			for(int i = 0; i < 4; i++)
			{
				ocr[i] = receive_byte();
			}

			if(ocr_buffer == NULL)
				return 0xFF;

			*ocr_buffer = ocr[0] << 24 | ocr[1] << 16 | ocr[2] << 8 | ocr[3];
		}

		return res;

	}
	else
	{
		return res;			//Return 0xFF if card is never ready
	}
}

/*
 * Description: Read a single data block
 * Param: 	*buff: pointer to buffer to store data
 * 			buffsize: size of the buffer (should be 512)
 * Return: 	Returns outcome, fail or success
 */
static STATUS read_block(uint8_t *buff, uint16_t buffsize)
{
	uint8_t token = 0xFF;

	start_timer();

	/* WAIT FOR CORRECT TOKEN */
	while(token == 0xFF && get_timer_val() < 500)
	{
		token = receive_byte();
	}

	if(token != 0xFE)
		return FAIL;

	/* RECEIVE DATA BLOCK TO BUFFER */
	uint16_t bytesLeft = buffsize;

	while(bytesLeft)
	{
		*(buff + (buffsize - bytesLeft)) = receive_byte();
		bytesLeft--;
	}

	/* RECEIVE AND DISCARD CRC */
	receive_byte();
	receive_byte();

	return SUCCESS;
}

/*
 * Description: Write a single data block
 * Param: 	*buff: pointer to buffer with data to write
 * 			buffsize: size of the buffer (should be 512)
 * Return: 	Returns outcome, fail or success
 */
static STATUS write_block(uint8_t *buff, uint16_t buffsize, uint8_t *token)
{
	/* MAKE SURE CARD IS READY */
	wait_sd_ready();

	/* SEND TOKEN */
	exchange_byte(token);

	/* SEND DATA BLOCK */
	uint16_t bytesLeft = buffsize;

	while(bytesLeft)
	{
		exchange_byte(buff + (buffsize - bytesLeft));
		bytesLeft--;
	}

	/* SEND CRC */
	uint8_t dummyCRC = 0;
	exchange_byte(&dummyCRC);
	exchange_byte(&dummyCRC);

	/* RECEIVE DATA RESPONSE */
	uint8_t dataResponse;
	dataResponse = receive_byte();

	wait_sd_ready();

	if((dataResponse & 0x0F) == 5)
		return SUCCESS;

	return FAIL;
}

/*
 * Description: Returns disk status
 * Param: 	None
 * Return:	Disk status
 */
DSTATUS status_sd()
{
	return status;
}

/*
 * Description: Initialize the SD Card for SPI operation
 * Param: 	None
 * Return:	Disk status
 */
DSTATUS initialize_sd()
{
	/* ENTER NATIVE MODE */
	deselect();

	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	HAL_SPI_Init(&hspi1);
	HAL_Delay(1);

	uint8_t dummyData = 0xFF;

	for(int i = 0; i < 100; i++)
	{
		exchange_byte(&dummyData);
	}

	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	HAL_SPI_Init(&hspi1);
	HAL_Delay(1);
	select();

	/* ENTER SPI MODE */
	uint8_t res;

	res = send_command(CMD0, 0, 100, NULL);

	/* COMPLETE INITIALIZATION PROCESS */
	if(res == 0x01)
	{
		uint32_t ocr = 0;
		res = send_command(CMD8, 0x000001AA, 100, &ocr);

		/* BRANCH FOR SD VERSION 1 & MMC VERSION 3 */
		if(res != 0x01)
		{
			send_command(CMD55, 0, 10, NULL);
			res = send_command(ACMD41, 0, 10, NULL);

			uint16_t timeout = 10000;
			while(res == 0x01 && timeout)				//Try to initiate as SD Ver. 1
			{
				timeout--;
				send_command(CMD55, 0, 10, NULL);
				res = send_command(ACMD41, 0, 10, NULL);
			}

			if(res != 0)								//Try for MMC Ver. 3
			{
				send_command(CMD1, 0, 10, NULL);
				timeout = 10000;
				while(res == 0x01 && timeout)			//Try to initiate as MMC Ver. 3
				{
					timeout--;
					res = send_command(CMD1, 0, 10, NULL);
				}

				if(res != 0)							//Card could not be identified
				{
					version = 0;
					status = STA_NOINIT;
					deselect();
					return status;
				}

				version = 1;					//Card is MMC Ver. 3
			}
			else
			{
				version = 2;					//Card is SD Ver. 1
			}

			res = send_command(CMD16, 0x00000200, 10, NULL); // Force 512 byte block size

			if(res == 0)
				status &= ~STA_NOINIT;					//Set status to initiated

			deselect();
			return status;
		}
		/* BRANCH FOR SD VERSION 2+ */
		if((ocr & 0x00000FFF) != 0x1AA)					//Only continue if correct OCR response
		{
			version = 0;
			status = STA_NOINIT;
			deselect();
			return status;
		}

		uint16_t timeout = 10000;
		while(res == 0x01 && timeout)					//Try to initiate as SD Ver. 2+
		{
			timeout--;
			send_command(CMD55, 0, 10, NULL);
			res = send_command(ACMD41, 0x40000000, 10, NULL);
		}

		if(timeout == 0)
		{
			version = 0;
			status = STA_NOINIT;
			deselect();
			return status;
		}

		res = send_command(CMD58, 0, 10, &ocr);			//Read OCR register

		if(res == 0 && !(ocr & 0x40000000))
		{
			res = send_command(CMD16, 0x00000200, 10, NULL); // Force 512 byte block size
		}

		if(res == 0)
		{
			version = 3;
			status &= ~STA_NOINIT;					//Set status to initiated
		}
	}

	deselect();
	return status;
}

/*
 * Description: Read data from SD card
 * Param: 	pdrv: Disk number, must be zero in single disk system
 * 			*buff: Pointer to buffer to receive data
 * 			sector: Sector to start reading from
 * 			count: Number of sectors to read
 * Return: 	Returns outcome of operation
 */
DRESULT read_sd(BYTE pdrv, BYTE *buff, uint32_t sector, UINT count)
{
	/* MAKE SURE PARAMETERS ARE OK */
	if(status == STA_NOINIT)
		return RES_NOTRDY;

	if(pdrv || !count)
		return RES_PARERR;

	select();

	uint8_t res;
	wait_sd_ready();

	/* BRANCH FOR SINGLE BLOCK READ */
	if(count == 1)
	{
		res = send_command(CMD17, sector, 10, NULL);
		if(res == 0)
		{
			if(read_block(buff, 512))
			{
				deselect();
				return RES_OK;
			}
		}
	}
	else
	{
		/* BRANCH FOR MULTIPLE BLOCK READ */
		uint32_t blocksLeft = count;
		res = send_command(CMD18, sector, 10, NULL);
		if(res == 0)
		{
			while(blocksLeft)
			{
				if(read_block((buff + 512 * (count - blocksLeft)), 512) == FAIL)	//Increase buff pointer by block size each itteration
				{
					deselect();
					return RES_ERROR;
				}

				blocksLeft--;
			}

			send_command(CMD12, 0, 10, NULL);
			res = receive_byte();				//Prior byte is stuff byte
			wait_sd_ready();

			if(res == 0)
			{
				deselect();
				return RES_OK;
			}
		}
	}
	deselect();
	return RES_ERROR;
}

/*
 * Description: Write data to SD card
 * Param: 	pdrv: Disk number, must be zero in single disk system
 * 			*buff: Pointer to buffer with data to write
 * 			sector: Sector to start writing to
 * 			count: Number of sectors to write
 * Return: 	Returns outcome of operation
 */
DRESULT write_sd(BYTE pdrv, const BYTE *buff, uint32_t sector, UINT count)
{
	if(status == STA_NOINIT)
			return RES_NOTRDY;

		if(pdrv || !count)
			return RES_PARERR;

	select();
	uint8_t res;
	uint8_t CMD24_token = 0xFE, CMD25_token = 0xFC, CMD25_token_stop = 0xFD;

	/* BRANCH FOR SINGLE BLOCK WRITE */
	if(count == 1)
	{
		res = send_command(CMD24, sector, 10, NULL);
		if(res == 0)
		{
			if(write_block(buff, 512, &CMD24_token))
			{
				deselect();
				return RES_OK;
			}
		}
	}
	else
	{
		/* BRANCH FOR MULTIPLE BLOCK WRITE */
		uint32_t blocksLeft = count;
		res = send_command(CMD25, sector, 10, NULL);
		if(res == 0)
		{
			while(blocksLeft)
			{
				if(!write_block((buff + 512 * (count - blocksLeft)), 512, &CMD25_token))
				{
					deselect();
					return RES_ERROR;
				}
				blocksLeft--;
			}

			exchange_byte(&CMD25_token_stop);
			receive_byte();
			wait_sd_ready();
			deselect();
			return RES_OK;
		}
	}

	deselect();
	return RES_ERROR;
}


DRESULT ioctl_sd(BYTE pdrv, BYTE cmd, void *buff)
{
	return RES_ERROR;
}




