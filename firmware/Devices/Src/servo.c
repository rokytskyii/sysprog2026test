#include "servo.h"

bool servo_init(servo_t *dev, const pwm_interface_t *pwm)
{
    if (!dev || !pwm)
        return false;
    dev->pwm = pwm;
    servo_set_position(dev, SERVO_ANGLE_0);
    return true;
}

void servo_set_position(servo_t *dev, servo_position_t pos)
{
    if (dev && dev->pwm)
    {
        dev->pwm->set_pulse_us((uint16_t)pos);
    }
}