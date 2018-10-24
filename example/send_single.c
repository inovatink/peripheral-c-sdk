// Inovatink Smart IoT Module - Peripheral Client C SDK
//
// This example code demonstrates the communication between
// peripheral and Smart IoT Modules.
//
// Author: Inovatink
//

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../driver/per-client.h"

#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0) {
            printf ("error %d from tcgetattr", errno);
            return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0) {
            printf ("error %d from tcsetattr", errno);
            return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0) {
        	printf ("error %d from tggetattr", errno);
            return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        	printf ("error %d setting term attributes", errno);
        }
}

//! Byte swap unsigned int
uint32_t swap_uint32 ( uint32_t val )
{
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF ); 
    return (val << 16) | (val >> 16);
}

int main(void) 
{
	char *portname = "/dev/ttyUSB0";
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf ("error %d opening %s: %s", errno, portname, strerror (errno));
		return 0;
	}

	set_interface_attribs (fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	//write (fd, "hello!\n", 7);           // send 7 character greeting

	//usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
	                                       // receive 25:  approx 100 uS per char transmit
	//char buf [100];
	//int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read

	sm_cli_config_t sm_cfg = {
		.uart_ctx = fd,
		.write_func = write,
		.read_func = read,
	};

	sm_cli_handle_t sm_client = sm_cli_init( &sm_cfg );

    for(;;) {

    	sm_cli_prep_packet(sm_client, 0x53);

    	uint32_t tmp_data = 0x87654321;
    	tmp_data = swap_uint32(tmp_data);
    	
    	sm_cli_add_sensor_packet(sm_client, AR_SENS1, tmp_data);

    	tmp_data = 0x12345678;
    	tmp_data = swap_uint32(tmp_data);
    	
    	sm_cli_add_sensor_packet(sm_client, AR_SENS2, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS3, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS4, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS5, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS6, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS7, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS8, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS9, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS10, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS11, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS12, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS13, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS14, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS15, tmp_data);

        tmp_data = 0x12345678;
        tmp_data = swap_uint32(tmp_data);
        
        sm_cli_add_sensor_packet(sm_client, AR_SENS16, tmp_data);

    	sm_cli_post_packet(sm_client);
        
        sleep(10);
    }
	return 0;
}