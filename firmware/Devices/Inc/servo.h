#ifndef SERVO_H
#define SERVO_H

#include "bus_interfaces.h"

typedef enum
{
    SERVO_ANGLE_0 = 1000,  // Засув розблоковано
    SERVO_ANGLE_90 = 1500, // Проміжний стан
    SERVO_ANGLE_180 = 2000 // Заблоковано / опломбовано
} servo_position_t;

typedef struct
{
    const pwm_interface_t *pwm;
} servo_t;

bool servo_init(servo_t *dev, const pwm_interface_t *pwm);
void servo_set_position(servo_t *dev, servo_position_t pos);

#endif /* SERVO_H */