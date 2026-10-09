#ifndef SERVO_H
#define SERVO_H

#include <unistd.h>


void Servo(int ch, int deg); // ch defines channel range [0  1]. deg defines degree range [0  180]

void servo_start_pos();
void servo_down();
void servo_up();

#endif