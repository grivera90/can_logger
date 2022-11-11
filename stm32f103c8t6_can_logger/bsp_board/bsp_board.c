/**
*******************************************************************************
* @file           : bsp_bootloader.c
* @brief          : Description of C implementation module
* @author         : Gonzalo Rivera
* @date           : 30/08/2022
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
/******************************************************************************
    Includes
******************************************************************************/
#include "bsp_board.h"

#include <string.h>
#include "stdbool.h"

#include "stm32f1xx_hal.h"
/******************************************************************************
    Defines and constants
******************************************************************************/
#define BSP_MODULE "BSP_BOOTLOADER_v1.0.0"
#define UART_TX_TIMEOUT 5000
/******************************************************************************
    Data types
 ******************************************************************************/

/******************************************************************************
    Local variables
 ******************************************************************************/
static CAN_HandleTypeDef *can_handler;
static UART_HandleTypeDef *uart_handler;

/******************************************************************************
    Local function prototypes
 ******************************************************************************/
static bsp_ret_t bsp_can_create_filter(CAN_HandleTypeDef *hcan);

/******************************************************************************
    Local function definitions
******************************************************************************/
static bsp_ret_t bsp_can_create_filter(CAN_HandleTypeDef *hcan)
{
	if(NULL == (CAN_HandleTypeDef *)hcan)
		return BSP_ERROR;

	bsp_ret_t ret = BSP_SUCCESS;
	CAN_FilterTypeDef sFilterConfig;
	uint32_t filter_id 		= 0x0000CEF0;
	uint32_t filter_mask 	= 0x0000CEF0;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	sFilterConfig.FilterIdHigh = (filter_id << 3) >> 16;
	sFilterConfig.FilterIdLow = (0xFFFF & (filter_id << 3)) | (1 << 2);

	sFilterConfig.FilterMaskIdHigh = (filter_mask << 3) >> 16;
	sFilterConfig.FilterMaskIdLow = (0xFFFF & (filter_mask << 3)) | (1 << 2);

	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 1; // 14; // chequear este numerito si se asigna al CAN1 o CAN2 y como funciono

	if (HAL_OK != HAL_CAN_ConfigFilter(hcan, &sFilterConfig))
		return BSP_ERROR;

	return ret;
}

/******************************************************************************
    Public function definitions
 ******************************************************************************/
/* uart related functions */
bsp_ret_t bsp_uart_init(uint32_t *uartx)
{
	bsp_ret_t ret = BSP_SUCCESS;

	if(NULL == (UART_HandleTypeDef *)uartx)
		return BSP_ERROR;

	uart_handler = (UART_HandleTypeDef *)uartx;

	return ret;
}

bsp_ret_t bsp_uart_deinit(uint32_t *uartx)
{
	bsp_ret_t ret = BSP_SUCCESS;

	uartx = NULL;
	uart_handler = NULL;

	return ret;
}

bsp_ret_t bsp_uart_write(uint8_t *data, uint16_t size)
{
	bsp_ret_t ret = BSP_SUCCESS;

	 if(HAL_OK != HAL_UART_Transmit(uart_handler, data, size, UART_TX_TIMEOUT))
		 return BSP_ERROR;

	return ret;
}

bsp_ret_t bsp_uart_read(uint8_t *data, uint16_t size, uint16_t *rxbytes, uint32_t timeout)
{
	bsp_ret_t ret = BSP_SUCCESS;

	if(HAL_OK != HAL_UART_Receive(uart_handler, data, size, timeout))
		return BSP_ERROR;

	*rxbytes = uart_handler->RxXferSize;

	return ret;
}
/* end uart related functions */
/* can related functions */
bsp_ret_t bsp_can_init(uint32_t *hcan)
{
	bsp_ret_t ret = BSP_SUCCESS;

	if(NULL == (CAN_HandleTypeDef *)hcan)
		return BSP_ERROR;

	can_handler = (CAN_HandleTypeDef *)hcan; // apunto al handler local al modulo.

	if(BSP_SUCCESS != bsp_can_create_filter(can_handler))
		return BSP_ERROR;

	if(HAL_OK != HAL_CAN_Start(can_handler))
		return BSP_ERROR;

	/* creamos la interrupcion por recepcion */
	/* Don't forget to check NVIC in CAN -> NVIC Settings -> CAN RX0 interrupt */
	if (HAL_OK != HAL_CAN_ActivateNotification(can_handler, CAN_IT_RX_FIFO0_MSG_PENDING))
		return BSP_ERROR;

	return ret;
}

bsp_ret_t bsp_can_tx(uint32_t id, uint8_t *tx_data, uint8_t dlc)
{
	bsp_ret_t ret = BSP_SUCCESS;
	CAN_TxHeaderTypeDef tx_header;

	if(NULL == tx_data || NULL == (CAN_HandleTypeDef *)can_handler)
		return BSP_ERROR;

	tx_header.DLC = dlc;										/* Here we are sending 8 bytes */
	tx_header.RTR = CAN_RTR_DATA;								/* Data frame */
	tx_header.IDE = CAN_ID_EXT;									/* We want to send an extended ID */
	tx_header.TransmitGlobalTime = DISABLE;
	tx_header.ExtId = id;
	tx_header.StdId = 0x00;

	uint32_t tx_mailbox;
	if(HAL_OK != HAL_CAN_AddTxMessage(can_handler, (CAN_TxHeaderTypeDef *)&tx_header, tx_data, &tx_mailbox))
		return BSP_ERROR;

	while(HAL_CAN_IsTxMessagePending(can_handler, tx_mailbox) != 0);

	return ret;
}

bool bsp_can_get_id_data(uint32_t *id, uint8_t *data, uint8_t *size)
{
	HAL_StatusTypeDef status = HAL_OK;
	CAN_RxHeaderTypeDef rx_header = {0};
	uint8_t rx_data[8] = {0};

	status = HAL_CAN_GetRxMessage(can_handler, CAN_RX_FIFO0, &rx_header, rx_data);

	if (HAL_OK != status)
	{
		// TODO: call error handler, while(1) for now.
		__disable_irq();
		while(1);
	}

	/* Check the length of the data */
	if(rx_header.DLC == 0)
		return false;

	/* Read ID */
	if(rx_header.IDE == CAN_ID_STD)
		*id = rx_header.StdId;
	else
		*id = rx_header.ExtId;

	/* Read data */
	memcpy(data, rx_data, rx_header.DLC);
	*size = rx_header.DLC;

	/* Leave the function by saying that the message is new */
	return true;
}
/* end can related functions*/
/* gpio related functions */
void bsp_gpio_write(uint32_t *gpio_port, uint16_t gpio_pin, bool value)
{
	HAL_GPIO_WritePin((GPIO_TypeDef *) gpio_port, gpio_pin, (GPIO_PinState) value);
}

bool bsp_gpio_read(uint32_t *gpio_port, uint16_t gpio_pin)
{
	return HAL_GPIO_ReadPin((GPIO_TypeDef *) gpio_port, gpio_pin);
}
/* end gpio related functions */
/* adc related functions */
uint32_t bsp_adc_read(uint32_t *adc, uint8_t channel)
{
	uint32_t adc_value = 0;

	return adc_value;
}
/* end ac related functions */
/* dac related functions */
void bsp_dac_set(uint32_t *dac, uint16_t value)
{

}
/* end dac related functions */
void bsp_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}
/* flash related functions */
bsp_ret_t bsp_flash_word(uint32_t address, uint32_t data)
{
	bsp_ret_t ret = BSP_SUCCESS;

	if(HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data))
		return BSP_ERROR;

	return ret;
}

bsp_ret_t bsp_flash_erase(uint32_t address_start, uint32_t npages)
{
	bsp_ret_t ret = BSP_SUCCESS;
	FLASH_EraseInitTypeDef erase_init = {0};
	uint32_t page_err = 0;

	erase_init.NbPages = npages;
	erase_init.PageAddress = address_start;
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;

	if(HAL_OK != HAL_FLASHEx_Erase(&erase_init, &page_err))
		return BSP_ERROR;

	return ret;
}

bsp_ret_t bsp_unlock_flash(void)
{
	bsp_ret_t ret = BSP_SUCCESS;

	if(HAL_OK != HAL_FLASH_Unlock())
		return BSP_ERROR;

	return ret;
}

bsp_ret_t bsp_lock_flash(void)
{
	bsp_ret_t ret = BSP_SUCCESS;

	if(HAL_OK != HAL_FLASH_Lock())
		return BSP_ERROR;

	return ret;
}
/* end flash related functions */
