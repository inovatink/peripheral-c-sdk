// Inovatink Smart IoT Module - Peripheral C SDK
//
// This is the Peripheral SDK that is created to
// communicate with Inovatink Smart IoT Modules.
//
// Author: Inovatink
//

/* -- Includes -- */
/* includes for standart integer types */
#include <stdint.h> /* for uint_x */

/*! \addtogroup 'SMPeripheralProtocol'
 *  Message type, action register and configuration register parameters
 *  indicated with definitions for SMPeripheralProtocol.
 *  @{
 */
// Message Types
#define MT_CONF          0x43 //!< Message Type - Configuration
#define MT_SENS          0x53 //!< Message Type - Sensor

// Action Registers
#define AR_SENS1         0x01 //!< Action Register - Sensor 1
#define AR_SENS2         0x02 //!< Action Register - Sensor 2
#define AR_SENS3         0x04 //!< Action Register - Sensor 3
#define AR_SENS4         0x08 //!< Action Register - Sensor 4
#define AR_SENS5         0x10 //!< Action Register - Sensor 5
#define AR_SENS6         0x20 //!< Action Register - Sensor 6
#define AR_SENS7         0x40 //!< Action Register - Sensor 7
#define AR_SENS8         0x80 //!< Action Register - Sensor 8
#define AR_SENS9         0x0100 //!< Action Register - Sensor 9
#define AR_SENS10        0x0200 //!< Action Register - Sensor 10
#define AR_SENS11        0x0400 //!< Action Register - Sensor 11
#define AR_SENS12        0x0800 //!< Action Register - Sensor 12
#define AR_SENS13        0x1000 //!< Action Register - Sensor 13
#define AR_SENS14        0x2000 //!< Action Register - Sensor 14
#define AR_SENS15        0x4000 //!< Action Register - Sensor 15
#define AR_SENS16        0x8000 //!< Action Register - Sensor 16
/*! @} */

#define DATA_BUFFER_LEN 128//!< Action Register - Sensor 1

typedef struct sm_handle* sm_handle_t;

typedef int (*write_func_ptr)(void *, uint8_t*, int);
typedef int (*read_func_ptr) (void *, uint8_t*, int);

/**
 * Smart IoT Module Peripheral Function Returns
 */
typedef enum {
	SM_FAIL = -1, /*!< FAIL enum parameter for Smart IoT Module Peripheral Error Type */
	SM_OK = 0, /*!< OK enum parameter for Smart IoT Module Peripheral Error Type */
} sm_err_t;

/**
 * Smart IoT Module Peripheral Packet Header structure
 * @brief Data structure that includes packet header of SM Peripheral Protocol. 
 */
typedef struct {
	uint16_t message_type; /*!< Packet Header Message Type */
	uint8_t p_len; /*!< Packet Header Payload Length */
	uint16_t action_reg; /*!< Packet Header Action Registers */
} p_hdr;

/**
 * Smart IoT Module Peripheral Configuration Structure
 * @brief Data structure that includes configuration parameters to initialize
 * such as an uart handle pointer, write & read function pointer
 * to link host uart driver methods. 
 */
typedef struct {
	void *uart_ctx;  /*!< uart context pointer for typical uart handle */
	write_func_ptr write_func; /*!< write function pointer to point host uart write method */
	read_func_ptr read_func; /*!< read function pointer to point host uart read method */
} sm_config_t;

/**
 * @brief Smart IoT Module Peripheral SDK - Initialization
 *
 *        This method initializes peripheral with the intended configuration structure,
 *        allocates memory for the handle and sets accordingly. It then returns handle.
 *
 * @param Smart IoT Module peripheral configuration structure
 *
 * @return
 *     - initialized Smart IoT Module peripheral handle
 */
sm_handle_t sm_init ( sm_config_t *config );

/**
 * @brief Smart IoT Module Peripheral SDK - Deleting Client
 *
 *        This method deletes Peripheral by freeing memory allocated previously both
 *        for handle and data buffer. It then returns SM_OK.
 *
 * @param handle Smart IoT Module peripheral handle.
 *
 * @return
 *     - SM_OK If everything occurs as expected.
 */
sm_err_t sm_delete ( sm_handle_t handle );

/**
 * @brief Smart IoT Module Peripheral SDK - Prepare Protocol Packet
 *
 *        This method takes handle and message type and prepares protocol packet by
 *        changing message type in the packet header. It then returns SM_OK if a valid
 *        message type is given as input. If an unevaluated type is passed, it returns
 *        SM_FAIL.
 *
 * @param handle Smart IoT Module peripheral handle.
 * @param message_type Peripheral Protocol message type.
 *
 * @return
 *     - SM_OK If a valid message type is passed.
 *     - SM_FAIL If an invalid message type is passed.
 */
sm_err_t sm_prep_packet ( sm_handle_t handle, uint8_t message_type );

/**
 * @brief Smart IoT Module Peripheral SDK - Add Sensor Data to Protocol Packet
 *
 *        This method takes handle, action register, that indicates the sensor number and
 *        sensor data. Action register value is checked whether it is valid or not and then
 *        OR-ed with the current action register of the protocol packet. Sensor data is concatenated
 *        to the data buffer and protocol packet payload length is updated accordingly. 
 *        IMPORTANT NOTE: Sensor values should be prepared with ascending order of sensor number, S0,
 *        S1, S2 ... After that sm_post_packet should be called to send it to the sm.  
 *
 * @param handle Smart IoT Module peripheral handle.
 * @param action_reg Peripheral Protocol Action Register of current sensor
 * @param sens_data Data of the sensor with 'action_reg' action register
 * 
 * @return
 *     - SM_OK If a valid action register is passed.
 *     - SM_FAIL If an invalid action register is passed.
 */
sm_err_t sm_add_sensor_packet ( sm_handle_t handle, uint16_t action_reg, uint32_t sens_data );

/**
 * @brief Smart IoT Module Peripheral SDK - Post Protocol Packet
 *
 *        This method takes handle and transmits packet header and payload according
 *        to the Smart IoT Module peripheral protocol. It then resets the packet header
 *        and payload.
 *
 * @param handle Smart IoT Module peripheral handle.
 * 
 * @return
 *     - SM_OK If everything occurs as expected.
 */
sm_err_t sm_post_packet ( sm_handle_t handle );
