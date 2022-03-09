//------------------------------------------------------------
// Collection of Pixy Functions for 2022 FRC Robot
// First developed for the 20 and 21 FRC Compotition
//------------------------------------------------------------
#ifndef PIXY_STUFF_H_
#define PIXY_STUFF_H_

#include <stdint.h>
#include <string>

#include "defines.h"
//#include "trajectory.h"

#define SERVO_ADJUSTMENT_STEP_SIZE   0.03

using namespace std;


void SetBallColorIndex(int ballColorIndex);
void PixyStuffInit(string nt_table_name);
int PixyProcessData (int n_bytes_read, char uartbuffer[]);
void BallSpeed();


#endif // PIXY_STUFF_H_