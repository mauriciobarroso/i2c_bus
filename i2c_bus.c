/**
  ******************************************************************************
  * @file           : i2c_bus.c
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

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "i2c_bus.h"
#include "esp_log.h"

/* Private macros ------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "i2c_bus";

/* Private function prototypes -----------------------------------------------*/
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
static int8_t i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t length, void *intf_ptr);

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
static int8_t i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t length, void *intf_ptr);

/* Exported functions definitions --------------------------------------------*/
esp_err_t i2c_bus_init(i2c_bus_t *const me, int i2c_num, int sda_gpio,
		int scl_gpio, bool sda_pullup, bool scl_pullup, uint32_t clk_speed) {
	esp_err_t ret = ESP_OK;

	/* Fill structure members */
	me->conf.sda_gpio = sda_gpio;
	me->conf.scl_gpio = scl_gpio;
	me->conf.sda_pullup = sda_pullup;
	me->conf.scl_pullup = scl_pullup;
	me->conf.clk_speed = clk_speed;
	me->conf.num = i2c_num;
	me->devs.dev = NULL;
	me->devs.num = 0;

	/* Configure I2C parameters */
	i2c_config_t i2c_conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = me->conf.sda_gpio,
			.scl_io_num = me->conf.scl_gpio,
			.sda_pullup_en = me->conf.sda_pullup,
			.scl_pullup_en = me->conf.scl_pullup,
			.master.clk_speed = me->conf.clk_speed
	};

  ret = i2c_param_config(me->conf.num, &i2c_conf);

  if (ret != ESP_OK) {
  	ESP_LOGE(TAG, "Failed to configure I2C parameters");
  	return ret;
  }

  /* Install I2C driver */
  ret = i2c_driver_install(me->conf.num, I2C_MODE_MASTER, 0, 0, 0);

  if (ret != ESP_OK) {
  	ESP_LOGE(TAG, "Failed to install I2C driver");
  	return ret;
  }

	/* return ESP_OK */
	return ret;
}

esp_err_t i2c_bus_add_dev(i2c_bus_t *const me, uint8_t dev_addr, const char* name,
		i2c_bus_read_t read, i2c_bus_write_t write) {
	/* Print information message */
	ESP_LOGI(TAG, "Adding %s device to bus...", name ? name : "other");

	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Allocate memory for a new device */
	me->devs.dev = (i2c_bus_dev_t *)realloc(me->devs.dev, ++(me->devs.num) * sizeof(i2c_bus_dev_t));

	/* Fill structure members */
	if (me->devs.dev) {
		me->devs.dev[me->devs.num - 1].i2c_num = me->conf.num;
		me->devs.dev[me->devs.num - 1].dev_num = me->devs.num - 1;
		me->devs.dev[me->devs.num - 1].addr = dev_addr;
		me->devs.dev[me->devs.num - 1].read = read ?	read : i2c_bus_read;
		me->devs.dev[me->devs.num - 1].write = write ?	write : i2c_bus_write;
		strcpy(me->devs.dev[me->devs.num - 1].name, name ? name : "other");
	}
	else {
		ESP_LOGE(TAG, "Failed to allocate memory for a device");
		return ESP_ERR_NO_MEM;
	}

	/* Print successful message */
	ESP_LOGI(TAG, "Device added successfully");

	/* Return ESP_OK */
	return ret;
}

/* Private function definitions ----------------------------------------------*/
static int8_t i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t length, void *intf_ptr) {
	esp_err_t err = ESP_OK;
	int8_t rslt = I2C_BUS_OK;
	i2c_bus_dev_t *comm = NULL;

#ifdef I2C_BUS_BUFFER_SIZE
	if (length > I2C_BUS_BUFFER_SIZE) {
		return I2C_BUS_E_COM_FAIL;
	}
#endif

	if (intf_ptr) {
		comm = (i2c_bus_dev_t *)intf_ptr;

    err = i2c_master_write_read_device(comm->i2c_num, comm->addr, /* fixme: generalize it */
    		&reg_addr, 1, reg_data, length, 1000 / portTICK_PERIOD_MS);

    if (err != ESP_OK) {
    	rslt = I2C_BUS_E_COM_FAIL;
    }
	}
	else {
		rslt = I2C_BUS_E_NULL_PTR;
	}

	return rslt;
}

static int8_t i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t length, void *intf_ptr) {
  uint32_t i;
	esp_err_t err = ESP_OK;
	int8_t rslt = I2C_BUS_OK;
	i2c_bus_dev_t *comm = NULL;

#ifdef I2C_BUS_BUFFER_SIZE
	if (length + 1 > I2C_BUS_BUFFER_SIZE) {
		return I2C_BUS_E_COM_FAIL;
	}
#endif

	if (intf_ptr) {
		comm = (i2c_bus_dev_t *)intf_ptr;

		/* I2C write */
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
    	i2c_cmd_link_delete_static(handle);
    }

    err = i2c_master_write_byte(handle, comm->addr << 1 | I2C_MASTER_WRITE, true); /* fixme: generalize it */
    if (err != ESP_OK) {
    	i2c_cmd_link_delete_static(handle);
    }

    err = i2c_master_write_byte(handle, reg_addr, true);

    for (i = 0; i < length; i++) {
    	err = i2c_master_write_byte(handle, reg_data[i], true);
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(comm->i2c_num, handle, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(handle);

    if (err != ESP_OK) {
    	rslt = I2C_BUS_E_COM_FAIL;
    }
	}
	else {
		rslt = I2C_BUS_E_NULL_PTR;
	}

	return rslt;
}

/***************************** END OF FILE ************************************/
