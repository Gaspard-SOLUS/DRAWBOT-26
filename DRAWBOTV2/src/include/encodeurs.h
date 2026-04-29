#pragma once
#include <Arduino.h>

void initEncoders();

long getLeftEncoderTicks();
long getRightEncoderTicks();

void resetLeftEncoder();
void resetRightEncoder();
void resetEncoders();

void updateEncoderMeasurements(unsigned long nowMs);

float getLeftDistanceCm();
float getRightDistanceCm();
float getAverageDistanceCm();

float getLeftSpeedCmParSec();
float getRightSpeedCmParSec();
float getLeftSpeedTicksParSec();
float getRightSpeedTicksParSec();