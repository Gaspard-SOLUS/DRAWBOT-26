#pragma once

void startAvanceForwardDistance(float targetCm, int cruisePwm, int slowPwm, float slowZoneCm);
void updateAvance();

bool isAvanceTermine();
float getAvanceTargetCm();
float getAvanceCurrentCm();