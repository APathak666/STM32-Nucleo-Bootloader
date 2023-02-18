#include <stdio.h>
#include "flash_update.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

/* Buffer to hold the received data */
static uint8_t Rx_Buffer[ ETX_OTA_PACKET_MAX_SIZE ];

/* OTA State */
static ETX_OTA_STATE_ ota_state = ETX_OTA_STATE_IDLE;

/* Firmware Total Size that we are going to receive */
static uint32_t ota_fw_total_size;
/* Firmware image's CRC32 */
static uint32_t ota_fw_crc;
/* Firmware Size that we have received */
static uint32_t ota_fw_received_size;

static uint16_t etx_receive_chunk(uint8_t *buf, uint16_t max_len);
static ETX_OTA_EX_ etx_process_data(uint8_t *buf, uint16_t len);
static void etx_ota_send_resp(uint8_t type);
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint16_t data_len, bool is_first_block);

ETX_OTA_EX_ etx_ota_download_and_flash(void)
{
	ETX_OTA_EX_ ret  = ETX_OTA_EX_OK;
	uint16_t    len;

	printf("Waiting for the OTA data...\r\n");

	/* Reset the variables */
	ota_fw_total_size    = 0u;
	ota_fw_received_size = 0u;
	ota_fw_crc           = 0u;
	ota_state            = ETX_OTA_STATE_START;

	do
	{
		memset( Rx_Buffer, 0, ETX_OTA_PACKET_MAX_SIZE );
		len = etx_receive_chunk( Rx_Buffer, ETX_OTA_PACKET_MAX_SIZE );

		if (len == 0u)
			ret = ETX_OTA_EX_ERR;
		else
			ret = etx_process_data( Rx_Buffer, len );

		if (ret == ETX_OTA_EX_OK)
			etx_ota_send_resp( ETX_OTA_ACK );

		else
		{
			printf("Sending NACK\r\n");
			etx_ota_send_resp( ETX_OTA_NACK );
			break;
		}

	} while (ota_state != ETX_OTA_STATE_IDLE);

	return ret;
}

static uint16_t etx_receive_chunk(uint8_t *buf, uint16_t max_len)
{
	int16_t  ret;
	uint16_t index = 0u;
	uint16_t data_len;

	do
	{
		//receive SOF (1 byte)
		if ((ret = HAL_UART_Receive( &huart2, &buf[index], 1, HAL_MAX_DELAY )) != HAL_OK)
			break;

		//verify SOF received
		if (buf[index++] != ETX_OTA_SOF)
		{
			ret = ETX_OTA_EX_ERR;
			break;
		}

		//receive packet type (1 byte)
		if ((ret = HAL_UART_Receive( &huart2, &buf[index++], 1, HAL_MAX_DELAY )) != HAL_OK)
			break;

		//packet data length (2 byte)
		if ((ret = HAL_UART_Receive( &huart2, &buf[index], 2, HAL_MAX_DELAY )) != HAL_OK)
			break;

		data_len = * (uint16_t*) &buf[index];
		index += 2u;

		//receive data
		for (uint16_t i = 0; i < data_len; i++)
			if ((ret = HAL_UART_Receive(&huart2, &buf[index++], 1, HAL_MAX_DELAY)) != HAL_OK)
				break;

		//CRC
		if ((ret = HAL_UART_Receive(&huart2, &buf[index], 4, HAL_MAX_DELAY)) != HAL_OK)
			break;

		index += 4u;

		//EoF
		if ((ret = HAL_UART_Receive(&huart2, &buf[index], 1, HAL_MAX_DELAY)) != HAL_OK)
			break;

		if (buf[index++] != ETX_OTA_SOF)
		{
			ret = ETX_OTA_EX_ERR;
			break;
		}
	} while (0);

	if (ret != HAL_OK)
		index = 0u;

	if (max_len < index)
	{
		printf("Received more data than expected... Expected: %d | Received: %d\n\r", max_len, index);
		index = 0u;
	}

	return index;
}

static ETX_OTA_EX_ etx_process_data( uint8_t *buf, uint16_t len )
{
	ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

	do
	{
		if (buf == NULL || len == 0u)
			break;

		//check for OTA abort command
		ETX_OTA_COMMAND_* check = (ETX_OTA_COMMAND_*)buf;
		if (check->packet_type == ETX_OTA_PACKET_TYPE_CMD)
			if (check->cmd == ETX_OTA_CMD_ABORT)
			{
				printf("Received OTA abort command\n\r");
				break;
			}

		switch(ota_state)
		{
			case ETX_OTA_STATE_IDLE:
			{
				printf("ETX_OTA_STATE_IDLE...\n\r");
				ret = ETX_OTA_EX_OK;
			}
			break;

			case ETX_OTA_STATE_START:
			{
				ETX_OTA_COMMAND_* check = (ETX_OTA_COMMAND_*)buf;
				if (check->packet_type == ETX_OTA_PACKET_TYPE_CMD)
					if (check->cmd == ETX_OTA_CMD_START)
					{
						printf("Received OTA start command\n\r");
						ota_state = ETX_OTA_STATE_HEADER;
						ret = ETX_OTA_EX_OK;
					}
			}
			break;

			case ETX_OTA_STATE_HEADER:
			{
				ETX_OTA_HEADER_* check = (ETX_OTA_HEADER_*)buf;
				if (check->packet_type == ETX_OTA_PACKET_TYPE_HEADER)
				{
			          ota_fw_total_size = check->meta_data.package_size;
			          ota_fw_crc = check->meta_data.package_crc;
			          printf("Received OTA Header. FW Size = %ld\r\n", ota_fw_total_size);
			          ota_state = ETX_OTA_STATE_DATA;
			          ret = ETX_OTA_EX_OK;
				}
			}
			break;

			case ETX_OTA_STATE_DATA:
			{
				ETX_OTA_DATA_* check = (ETX_OTA_DATA_*)buf;
				uint16_t data_len = check->data_len;
				HAL_StatusTypeDef end;

				if (check->packet_type == ETX_OTA_PACKET_TYPE_DATA)
					if ((end = write_data_to_flash_app(buf, data_len, (ota_fw_received_size == 0))) == HAL_OK)
					{
						printf("[%ld/%ld]\r\n", ota_fw_received_size/ETX_OTA_DATA_MAX_SIZE, ota_fw_total_size/ETX_OTA_DATA_MAX_SIZE);

						if (ota_fw_received_size >= ota_fw_total_size)
							ota_state = ETX_OTA_STATE_END;
						ret = ETX_OTA_EX_OK;
					}
			}
			break;

			case ETX_OTA_STATE_END:
			{
				ETX_OTA_COMMAND_* check = (ETX_OTA_COMMAND_*)buf;
				if (check->packet_type == ETX_OTA_PACKET_TYPE_CMD)
					if (check->cmd == ETX_OTA_CMD_END)
					{
						printf("Received OTA end command\n\r");
						ota_state = ETX_OTA_STATE_IDLE;
						ret = ETX_OTA_EX_OK;
					}
			}
			break;

			default:
				ret = ETX_OTA_EX_ERR;
			break;
		}
	} while (0);

	return ret;
}

static void etx_ota_send_resp( uint8_t type )
{
	ETX_OTA_RESP_ rsp =
	{
		.sof         = ETX_OTA_SOF,
		.packet_type = ETX_OTA_PACKET_TYPE_RESPONSE,
		.data_len    = 1u,
		.status      = type,
		.crc         = 0u,                //TODO: Add CRC
		.eof         = ETX_OTA_EOF
	};

  //send response
	HAL_UART_Transmit(&huart2, (uint8_t *)&rsp, sizeof(ETX_OTA_RESP_), HAL_MAX_DELAY);
}

static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint16_t data_len, bool is_first_block)
{
	HAL_StatusTypeDef ret;

	do
	{
		if ((ret = HAL_FLASH_Unlock()) != HAL_OK)
		{
			printf("Error unlocking flash\r\n");
			break;
		}

		if (is_first_block)
		{
			printf("Erasing the Flash memory...\r\n");
			//Erase the Flash
			FLASH_EraseInitTypeDef EraseInitStruct;
			uint32_t SectorError;

			EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
			EraseInitStruct.Sector        = FLASH_SECTOR_5;
			EraseInitStruct.NbSectors     = 2;                    //erase 2 sectors(5,6)
			EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

			if ((ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError)) != HAL_OK)
				break;
		}

		for (uint16_t i = 0; i < data_len; i++)
		{
			if ((ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, ETX_APP_FLASH_ADDR + ota_fw_received_size, data[i])) != HAL_OK)
			{
				printf("Error flashing\r\n");
				break;
			}

			else
				ota_fw_received_size++;
		}

		if (ret != HAL_OK)
			break;

		if ((ret = HAL_FLASH_Lock()) != HAL_OK)
			break;
	} while (0);

	return ret;
}
