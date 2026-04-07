#pragma once

// User led
#define LEDU1 25
#define LEDU2 26

// Enable moteurs droit et gauche
#define EN_D 4
#define EN_G 23

// Commande PWM moteur droit
#define IN_1_D 16
#define IN_2_D 17

// Commande PWM moteur gauche
#define IN_1_G 19
#define IN_2_G 18

// Encodeur gauche
#define ENC_G_CH_A 27
#define ENC_G_CH_B 14

// Encodeur droit
#define ENC_D_CH_A 32
#define ENC_D_CH_B 33

// I2C
#define SDA 21
#define SCL 22

// Adresse I2C
#define ADDR_IMU 0x6B
#define ADDR_MAG 0x1E