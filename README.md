# SM Peripheral SDK

SDK for communication [Inovatink's](www.inovatink.com) wib series Smart IoT Modules.

## Installation

1. [Download](https://github.com/inovatink/peripheral-c-sdk/releases) the Latest release from GitHub.
2. Unzip and place folder contents into appropriate place inside your C project.

## Getting Started

Please refer to the example in the downloaded folder. At the begining of C application code include peripheral SDK:

```c++
#include "../sdk/per-client.h"
```

Use of this SDK assumes that reliable UART communication is established between application MCU and Smart IoT Module.

After creating serial port context in your C program populate following struct:

```c++
sm_config_t sm_cfg = {
    .uart_ctx = fd,       // Point context of UART Interface here
    .write_func = write,  // Point write function of UART Interface here
    .read_func = read,    // Point read function of UART Interface here
};
```

To start using SDK it needs to be initialized:

```c++
// Smart Module Peripheral SDK Initialization returns a handle that is used to call
// any methods from the SDK.
sm_handle_t sm_client = sm_init( &sm_cfg );
```

After initialization data can be sent to cloud through module. This process consist of preparing message, adding sensor data and posting message.

Prepare:

```c++
// In order to prepare a message packet, message type should be passed to the 
// sm_prep_packet method.
sm_prep_packet(sm_client, 0x53);
```

Add:

```c++
// After reading or generating sensor data, it should be passed to the sdk using
// sm_add_sensor_packet method in order to form the message packet structure according to 
// the SDK. Keep in mind to pass the values & sensors in an order such as S1, S2, ... S10.

// First packed sensor index should be larger than the one that used next. 
sm_add_sensor_packet(sm_client, AR_SENS1, sensor1_data);

// Second sensor reading packed to the message.
sm_add_sensor_packet(sm_client, AR_SENS2, sensor2_data);
```

Post:

```c++
// Message packet is posted to the Smart Module. After sending, sdk cleans all the information 
// used for the sent message. To be able to send an other, packet type should be prepared using 
// sm_prep_packet method (There is no need to call sm_init again).
sm_post_packet(sm_client);
```