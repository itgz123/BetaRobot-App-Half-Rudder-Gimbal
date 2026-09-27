#ifndef __APP_CMD_H
#define __APP_CMD_H

#include <stdint.h>

void AppCmdInit(void);
void AppCmdRun(float dt, uint64_t time_stamp);

#endif // !__APP_CMD_H