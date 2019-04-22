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
#include "../sdk/per-client.h"

#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

// UART Interface Initialization
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
        tty.c_iflag &= ~IGNBRK;             // disable break processing
        tty.c_lflag = 0;                    // no signaling chars, no echo,
                                            // no canonical processing
        tty.c_oflag = 0;                    // no remapping, no delays
        tty.c_cc[VMIN]  = 0;                // read doesn't block
        tty.c_cc[VTIME] = 5;                // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
                                            // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
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
        tty.c_cc[VTIME] = 5;                // 0.5 seconds read timeout

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
    // Initialize UART parameters
    char *portname = "/dev/ttyUSB0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return 0;
    }

    set_interface_attribs (fd, B38400, 0);  // set speed to 38,400 bps, 8n1 (no parity)
    set_blocking (fd, 0);                    // set no blocking

    // Smart Module Peripheral SDK 
    sm_config_t sm_cfg = {
        .uart_ctx = fd,       // Point context of UART Interface here
        .write_func = write,  // Point write function of UART Interface here
        .read_func = read,    // Point read function of UART Interface here
    };

    // Smart Module Peripheral SDK Initialization returns a handle that is used to call
    // any methods from the SDK.
    sm_handle_t sm_client = sm_init( &sm_cfg );

    for(;;) {
        
    // In order to prepare a message packet, message type should be passed to the 
    // sm_prep_packet method.
    sm_prep_packet(sm_client, 0x53);

    uint32_t tmp_data = 0x4048f5c3; // 3.14
    tmp_data = swap_uint32(tmp_data);
    
    // After reading or generating sensor data, it should be passed to the sdk using
    // sm_add_sensor_packet method in order to form the message packet structure according to 
    // the SDK. Keep in mind to pass the values & sensors in an order such as S1, S2, ... S10.
    // First packed sensor index should be larger than the one that used next. 
    sm_add_sensor_packet(sm_client, AR_SENS1, tmp_data);

    tmp_data = 0x402d70a4; // 2.71
    tmp_data = swap_uint32(tmp_data);
    
    // Second sensor reading packed to the message.
    sm_add_sensor_packet(sm_client, AR_SENS2, tmp_data);

    // Message packet is posted to the Smart Module. After sending, sdk cleanups all the information 
    // used for the sent message. To be able to send an other, packet type should be prepared using 
    // sm_prep_packet method (There is no need to call sm_init again).
    sm_post_packet(sm_client);
        
        sleep(10);
    }
	return 0;
}