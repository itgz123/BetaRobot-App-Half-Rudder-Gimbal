#ifndef __APP_SENSOR_H
#define __APP_SENSOR_H

#include "app.h"

void AppSensorInit(void);
void AppSensorRun(void);

/* 最新视觉接收帧（app_sensor 接收回调同步更新；业务层只读引用） */
extern vision_recv_t vision_recv_data;

#endif // !__APP_SENSOR_H
