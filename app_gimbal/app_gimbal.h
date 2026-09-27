#ifndef __APP_GIMBAL_H
#define __APP_GIMBAL_H

#include <stdint.h>

void AppGimbalInit(void);
void AppGimbalRun(float dt, uint64_t time_stamp);

#endif // !__APP_GIMBAL_H
