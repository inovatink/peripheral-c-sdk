// Inovatink Smart IoT Module - Peripheral C SDK
//
// This is the Peripheral SDK that is created to
// communicate with Inovatink Smart IoT Modules.
//
// Author: Inovatink
//

#include "per-client.h"

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <errno.h>
#include <fcntl.h> 
#include <termios.h>

#ifdef DEBUG
  #define logf(...) printf(__VA_ARGS__)
#else
  #define logf(...)
#endif

uint16_t valid_arr[16] = {AR_SENS1, AR_SENS2, AR_SENS3, AR_SENS4, AR_SENS5, AR_SENS6, AR_SENS7, AR_SENS8, \
AR_SENS9, AR_SENS10, AR_SENS11, AR_SENS12, AR_SENS13, AR_SENS14, AR_SENS15};

static void byte_arr_2_hex_str ( uint8_t *byte_arr, char *hex_str, size_t byte_arr_size )
{
	for (int i = 0; i < byte_arr_size; i++) {
		hex_str += sprintf(hex_str, "%02x", *byte_arr);
		byte_arr++;
	}
}

static bool is_value_valid ( uint16_t action_reg )
{
    for (int i = 0; i < sizeof(valid_arr); i++) {
        if (valid_arr[i] == action_reg)

            return true;
    }

    return false;
}

//! Byte swap unsigned short
static uint16_t swap_uint16( uint16_t val ) 
{
    return (val << 8) | (val >> 8 );
}

struct sm_handle {
	p_hdr hdr;
	uint8_t *data_buffer;
	void *uart_ctx;
	write_func_ptr write_func;
	read_func_ptr read_func;
};

sm_handle_t sm_init ( sm_config_t *config )
{
	assert(config != NULL);
	sm_handle_t handle = calloc(1, sizeof(struct sm_handle));
	handle->data_buffer = calloc(1, DATA_BUFFER_LEN);
	handle->uart_ctx = config->uart_ctx;
	handle->write_func = config->write_func;
	handle->read_func = config->read_func;

	return handle;
}

sm_err_t sm_delete ( sm_handle_t handle )
{
	free(handle->data_buffer);
	free(handle);

	return SM_OK;
}

sm_err_t sm_prep_packet ( sm_handle_t handle, uint8_t message_type )
{
	assert(handle != NULL);
	if (message_type == 0x53) {
		handle->hdr.message_type = 0x5353;
	}
	else if (message_type == 0x43) {
		handle->hdr.message_type = 0x4343;
	}
	else {
		logf("wrong message type\r\n");

		return SM_FAIL;
	}

	return SM_OK;
}

sm_err_t sm_add_sensor_packet ( sm_handle_t handle, uint16_t action_reg, uint32_t sens_data )
{
	if (is_value_valid(action_reg)) {
		handle->hdr.action_reg |= action_reg;
		
		char *action_reg_str = calloc(1, 4 * sizeof(char) + 1);
		byte_arr_2_hex_str((uint8_t*)&handle->hdr.action_reg, action_reg_str, 2);
		logf("Updated Action Register: %s\r\n", action_reg_str);
		free(action_reg_str);
		action_reg_str = NULL;

		memcpy(&handle->data_buffer[handle->hdr.p_len], &sens_data, 4); 
		handle->hdr.p_len += 4;
		logf("Updated Packet Length: %d\r\n", handle->hdr.p_len);
		char *hex_str = calloc(1, (handle->hdr.p_len) * 2 * sizeof(char) + 1);
		byte_arr_2_hex_str(handle->data_buffer, hex_str, handle->hdr.p_len);
		logf("Updated Data Buffer: %s\r\n", hex_str);
		free(hex_str);
		hex_str = NULL;

		return SM_OK;
	}
	else {
		logf("invalid action register\r\n");

		return SM_FAIL;
	}
}

sm_err_t sm_post_packet ( sm_handle_t handle )
{
	handle->write_func(handle->uart_ctx, (uint8_t*)&handle->hdr.message_type, 2);
	handle->write_func(handle->uart_ctx, (uint8_t*)&handle->hdr.p_len, 1);
	uint16_t swapped_action_reg = swap_uint16(handle->hdr.action_reg);
	handle->write_func(handle->uart_ctx, (uint8_t*)&swapped_action_reg, 2);
	handle->write_func(handle->uart_ctx, handle->data_buffer, handle->hdr.p_len);
	// clearing message header and buffer after successfully sending data to the module.
	handle->hdr.message_type = 0x0000;
	handle->hdr.p_len = 0x00;
	handle->hdr.action_reg = 0x0000;
	memset(handle->data_buffer, 0, DATA_BUFFER_LEN);

	return SM_OK;
}