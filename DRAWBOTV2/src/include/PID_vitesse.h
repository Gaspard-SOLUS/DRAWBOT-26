#pragma once

void initPidVitesse();
void setSpeedTargetsCmPerSec(float leftTarget, float rightTarget);
void stopSpeedControl();
void setPidGains(float newKp, float newKi, float newKd);
void updatePidVitesse(float dtSec);

float getKp();
float getKi();
float getKd();

float getLeftTargetSpeedCmPerSec();
float getRightTargetSpeedCmPerSec();
float getLeftPidError();
float getRightPidError();
float getLeftPidOutput();
float getRightPidOutput();