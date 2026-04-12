#pragma once

void startAvanceForwardDistance(float targetCm, int cruisePwm, int slowPwm, float slowZoneCm, float kpStraight);

void updateAvance(unsigned long now);

bool isAvanceTermine();
float getAvanceTargetCm();
float getAvanceCurrentCm();

float getAvanceStraightErrorTicks();
int getAvanceLeftCommand();
int getAvanceRightCommand();