#pragma once

#include <stdint.h>

void initEncoders();

long getLeftEncoderTicks();
long getRightEncoderTicks();

void resetLeftEncoder();
void resetRightEncoder();
void resetEncoders();

void updateEncoderMeasurements(unsigned long nowMs);

// Distances
float getLeftDistanceCm();
float getRightDistanceCm();
float getAverageDistanceCm();

// Vitesses
float getLeftSpeedTicksParSec();
float getRightSpeedTicksParSec();
float getLeftSpeedCmParSec();
float getRightSpeedCmParSec();