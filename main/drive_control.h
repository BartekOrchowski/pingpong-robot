#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

bool drive_control_start(void);

void drive_control_submit(int32_t axis_y);

void drive_control_stop(void);

#endif