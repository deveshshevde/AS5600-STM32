
#ifndef AS5600_INCLUDED
#define AS5600_INCLUDED

#include <stdint.h>


/// AS5600 address
#define AS5600_SLAVE_ADDRESS			0x36
#define AS5600_SHIFTED_SLAVE_ADDRESS	0x6c
#define AS5600_I2C_TIMEOUT_DEFAULT		10


#define AS5600_REGISTER_ZMCO			0x00
#define AS5600_REGISTER_ZPOS_HIGH		0x01
#define AS5600_REGISTER_ZPOS_LOW		0x02
#define AS5600_REGISTER_MPOS_HIGH		0x03
#define AS5600_REGISTER_MPOS_LOW		0x04
#define AS5600_REGISTER_MANG_HIGH		0x05
#define AS5600_REGISTER_MANG_LOW		0x06
#define AS5600_REGISTER_CONF_HIGH		0x07
#define AS5600_REGISTER_CONF_LOW		0x08

#define AS5600_REGISTER_RAW_ANGLE_HIGH	0x0C
#define AS5600_REGISTER_RAW_ANGLE_LOW	0x0D
#define AS5600_REGISTER_ANGLE_HIGH		0x0E
#define AS5600_REGISTER_ANGLE_LOW		0x0F

#define AS5600_REGISTER_STATUS			0x0B
#define AS5600_REGISTER_AGC				0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH	0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW	0x1C
#define AS5600_REGISTER_BURN			0xFF





#define AS5600_DIR_CW					1
#define AS5600_DIR_CCW					2

/* AS5600 bit mask */
#define AS5600_12_BIT_MASK				(uint16_t)4095

#define AS5600_DEG_CONV 8.7890625e-2    /* 360/4096 */
#define AS5600_RAD_CONV 1.5339808e-3    /* 2pi/4096 */


typedef struct {
    I2C_HandleTypeDef*	i2c_handle;
    GPIO_TypeDef*		dir_port;
    uint32_t			i2c_timeout;
    uint16_t			dir_pin;
    uint8_t				positive_rotation_direction;



    uint8_t	config_register[2];
} AS5600_TypeDef;

HAL_StatusTypeDef	AS5600_init								(AS5600_TypeDef* handle);

HAL_StatusTypeDef 	AS5600_set_start_position				(AS5600_TypeDef* const handle, const uint16_t position);
HAL_StatusTypeDef	AS5600_set_stop_position				(AS5600_TypeDef* const handle, const uint16_t position);
HAL_StatusTypeDef	AS5600_set_max_angle					(AS5600_TypeDef* const handle, const uint16_t angle);
HAL_StatusTypeDef	AS5600_set_positive_rotation_direction	(AS5600_TypeDef* const handle, const uint8_t direction);
HAL_StatusTypeDef	AS5600_get_rawAngle						(AS5600_TypeDef* const handle, uint16_t* const angle);
HAL_StatusTypeDef	AS5600_get_angle						(AS5600_TypeDef* const handle, uint16_t* const angle);


#endif
