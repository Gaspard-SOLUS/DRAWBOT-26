#pragma once

void initMotors();
void setLeftMotor(int pwm);
void setRightMotor(int pwm);
void setMotors(int leftPwm, int rightPwm);
void stopMotors();
void brakeMotors();