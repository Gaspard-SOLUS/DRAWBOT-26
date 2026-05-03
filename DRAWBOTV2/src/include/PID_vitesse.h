#pragma once

void initPidVitesse();

void setSpeedTargetsCmPerSec(float leftTarget, float rightTarget);
void stopSpeedControl();

void updatePidVitesse(float dtSec);

int applyDeadzone(float output);

// PID gains
void setPidGains(float newKp, float newKi, float newKd);
float getKp();
float getKi();
float getKd();

float getLeftTargetSpeedCmPerSec();
float getRightTargetSpeedCmPerSec();

float getLeftPidError();
float getRightPidError();

float getLeftPidOutput();
float getRightPidOutput();