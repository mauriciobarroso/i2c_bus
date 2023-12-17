/**
  ******************************************************************************
  * @file           : i2c_bus.h
  * @author         : Mauricio Barroso Benavides
  * @date           : Jul 16, 2023
  * @brief          : todo: write brief 
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2023 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_BUS_H_
#define I2C_BUS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"

/* Exported Macros -----------------------------------------------------------*/
/* Return code definitions */
#define I2C_BUS_OK			INT8_C(0) /* Success */

/* Errors */
#define I2C_BUS_E_NULL_PTR				INT8_C(-1) /* Null pointer passed */
#define I2C_BUS_E_COM_FAIL				INT8_C(-2) /* Communication failure */
#define I2C_BUS_E_DEV_NOT_FOUND		INT8_C(-3) /* Sensor not found */
#define I2C_BUS_E_INVALID_LENGTH	INT8_C(-4) /* Incorrect length parameter */
#define I2C_BUS_E_SELF_TEST				INT8_C(-5) /* Self test fail error */

#define I2C_BUS_BUFFER_SIZE				(64)

#define I2C_BUS_DEV_NUM						(32)
#define I2C_BUS_DEV_NAME					(32)

/* Exported typedef ----------------------------------------------------------*/
/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param reg_addr : Pointer to register address of the device
 * @param addr_len : Length of the
 * @param reg_data : Data from the specified address
 * @param data_len : Length of the reg_data array
 * @param intf     : Void pointer that can enable the linking of descriptors
 *                   for interface related callbacks
 *
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef int8_t (*i2c_bus_read_t)(uint8_t *reg_addr, uint8_t addr_len,
		uint8_t *reg_data, uint32_t data_len, void *intf);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param reg_addr : Pointer to register address of the device
 * @param addr_len : Length of the
 * @param reg_data : Data from the specified address
 * @param data_len : Length of the reg_data array
 * @param intf     : Void pointer that can enable the linking of descriptors
 *                   for interface related callbacks
 *
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef int8_t (*i2c_bus_write_t)(uint8_t *reg_addr, uint8_t addr_len,
		const uint8_t *reg_data, uint32_t data_len, void *intf);

typedef int8_t i2c_bus_err_t;

/**
 * @brief Structure to store I2C bus configuration parameters
 */
typedef struct { /* todo: place in i2c_bus.h */
	int num;
	int sda_gpio;
	int scl_gpio;
	bool sda_pullup;
	bool scl_pullup;
	uint32_t clk_speed;
} i2c_bus_conf_t;

/**
 * @brief Structure to store the device parameter for an I2C device
 */
typedef struct {
	int *i2c_num;
	SemaphoreHandle_t *mutex;
	uint8_t dev_num;
	uint8_t addr;                /* Device address */
	char name[I2C_BUS_DEV_NAME]; /* Device name */
	i2c_bus_write_t write;       /* Function to perform an I2C write */
	i2c_bus_read_t read;         /* Function to perform an I2C read */
} i2c_bus_dev_t;

/**
 * @brief Structure to store the device parameter for an I2C device
 */
typedef struct {
	i2c_bus_dev_t *dev;
	uint8_t num;
} i2c_bus_devs_t;

/**
 * @brief I2C bus structure
 */
typedef struct {
	i2c_bus_conf_t conf; /* I2C bus configuration */
	SemaphoreHandle_t mutex;
	TimerHandle_t timer;
	i2c_bus_devs_t devs; /* Array of I2C devices attached to bus */
} i2c_bus_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a AT24CS0x instance
 *
 * @param me      : Pointer to a structure instance of at24cs0x_t
 * @param i2c_bus : Pointer to a structure with the data to initialize the
 * 								    	   sensor as a I2C device
 * @param addr    : I2C device address
 * @param read    : I2C read function pointer
 * @param write   : I2C write function pointer
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_init(i2c_bus_t *const me, int i2c_num, int sda_gpio,
		int scl_gpio, bool sda_pullup, bool scl_pullup, uint32_t clk_speed);

/**
 * @brief Function to initialize a AT24CS0x instance
 *
 * @param me      : Pointer to a structure instance of at24cs0x_t
 * @param i2c_bus : Pointer to a structure with the data to initialize the
 * 								    	   sensor as a I2C device
 * @param addr    : I2C device address
 * @param read    : I2C read function pointer
 * @param write   : I2C write function pointer
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_add_dev(i2c_bus_t *const me, uint8_t dev_addr, const char* name,
		i2c_bus_read_t read, i2c_bus_write_t write);

/**
 * @brief Function to initialize a AT24CS0x instance
 *
 * @param me      : Pointer to a structure instance of at24cs0x_t
 * @param i2c_bus : Pointer to a structure with the data to initialize the
 * 								    	   sensor as a I2C device
 * @param addr    : I2C device address
 * @param read    : I2C read function pointer
 * @param write   : I2C write function pointer
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_remove_dev(i2c_bus_t *const me);

/**
 * @brief Function to initialize a AT24CS0x instance
 *
 * @param me       : Pointer to a structure instance of at24cs0x_t
 * @param dev_addr : I2C device address
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_set_dev_addr(i2c_bus_t *const me, uint8_t dev_addr);

/**
 * @brief Function to initialize a AT24CS0x instance
 *
 * @param me       : Pointer to a structure instance of at24cs0x_t
 * @param dev_name : I2C write function pointer
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_set_dev_name(i2c_bus_t *const me, const uint8_t *dev_name);

/**
 * @brief Function that scans the I2C bus to detect all the devices connected
 *
 * @param me       : Pointer to a structure instance of at24cs0x_t
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_scan(i2c_bus_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_H_ */

/***************************** END OF FILE ************************************/
