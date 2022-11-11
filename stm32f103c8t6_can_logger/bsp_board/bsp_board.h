/**
*******************************************************************************
* @file           : bsp_bootloader.h
* @brief          : Description of header file
* @author         : Gonzalo Rivera
* @date           : 30/08/2022
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
#ifndef __BSP_BOOTLOADER_H__
#define __BSP_BOOTLOADER_H__
/******************************************************************************
        Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
        Constants
 ******************************************************************************/

/******************************************************************************
        Data types
 ******************************************************************************/
typedef enum
{
	BSP_SUCCESS,
	BSP_BUSY,
	BSP_ERROR,
	BSP_TIMEOUT
} bsp_ret_t;

typedef enum
{
	MENSAJE_CMD_UPGRADE = 0,
	MENSAJE_ERASE,
	CMD_WRITE,
	OPEN,
	UPDATING,
	BUSY,
	CLOSE,
	DELAY_ST_MACHINE,
	ERROR_ST_MACHINE,
	MAX_STATE_MACHINE
} states_message_t;
/******************************************************************************
        Public function prototypes
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
        uart related functions
 ******************************************************************************/
/**
 * @brief
 * Function to uart init
 *
 * @param uint8_t uartx
 *
 * @return
 * See \p bsp_ret_t return codes.
 */
bsp_ret_t bsp_uart_init(uint32_t *uartx);

/**
 * @brief
 * Function to deinit uart
 *
 * @param uint8_t uartx
 *
 * @return
 * See \p bsp_ret_t return codes.
 */
bsp_ret_t bsp_uart_deinit(uint32_t *uartx);

/**
 * @brief
 * Function to usart write
 *
 * @param const uint8_t * data, uint16_t size
 *
 * @return
 * See \p bsp_ret_t return codes.
 */
bsp_ret_t bsp_uart_write(uint8_t *data, uint16_t size);

/**
 * @brief
 * Function to usart read
 *
 * @param uint8_t * data, uint16_t size, uint32_t timeout
 *
 * @return
 * See \p bsp_ret_t return codes.
 */
bsp_ret_t bsp_uart_read(uint8_t *data, uint16_t size, uint16_t *rxbytes, uint32_t timeout);
/******************************************************************************
        can related functions
 ******************************************************************************/
/**
 * @brief
 * Function to init can module of uC
 *
 * @param uint32_t *hcan: pointer to handler struct of module can
 *
 * @return
 * See \p bsp_ret_t return codes.
 */
bsp_ret_t bsp_can_init(uint32_t *hcan);

/**
 * @brief
 * Function to can transmit
 *
 * @param id: id = PGN (could be PGNDA), tx_data: pointer to data to transmit, dlc: data length
 *
 * @return
 * See \p bsp_ret_t return codes.
 */
bsp_ret_t bsp_can_tx(uint32_t id, uint8_t *tx_data, uint8_t dlc);

/**
 * @brief
 * Function to get id and data received
 *
 * @param id: pointer to store id, data: pointer to store data
 *
 * @return true: new message, false: no message.
 */
bool bsp_can_get_id_data(uint32_t *id, uint8_t *data, uint8_t *size);
/******************************************************************************
        gpio´s related functions
 ******************************************************************************/
/**
 * @brief
 * Function to gpio write value
 *
 * @param uint32_t *gpio_port, uint16_t gpio_pin, bool value
 *
 * @return none
 */
void bsp_gpio_write(uint32_t *gpio_port, uint16_t gpio_pin, bool value);

/**
 * @brief
 * Function to gpio write value
 *
 * @param uint32_t *gpio_port, uint16_t gpio_pin, bool value
 *
 * @return bool, value of pin
 */
bool bsp_gpio_read(uint32_t *gpio_port, uint16_t gpio_pin);
/******************************************************************************
        adc related functions
 ******************************************************************************/

/**
 * @brief
 * Function to read adc value
 *
 * @param uint32_t *adc, uint8_t channel
 *
 * @return value of channel adc
 */
uint32_t bsp_adc_read(uint32_t *adc, uint8_t channel);
/******************************************************************************
        dac related functions
 ******************************************************************************/

/**
 * @brief
 * Function to set dac value
 *
 * @param uint32_t *dac, uint16_t value
 *
 * @return none
 */
void bsp_dac_set(uint32_t *dac, uint16_t value);

/**
 * @brief
 * Function to delay
 *
 * @param uint32_t data
 *
 * @return none
 */
void bsp_delay_ms(uint32_t ms);
/******************************************************************************
        flash related functions
 ******************************************************************************/
bsp_ret_t bsp_flash_word(uint32_t address, uint32_t data);
bsp_ret_t bsp_flash_erase(uint32_t address_start, uint32_t npages);
bsp_ret_t bsp_unlock_flash(void);
bsp_ret_t bsp_lock_flash(void);

#ifdef __cplusplus
}
#endif


#endif /* __BSP_BOOTLOADER_H__ */
