#include "../../SCS_servo/SCS_servo.h"
#include "../SCServo.h"

void loop(void)
{
	HAL_Delay(1000);
	ArmForceEnable(2,1);
	WritePos(2, 1000, 1000, 0);
	HAL_Delay(1000);
	WritePos(2, 0, 1000, 0);
}
