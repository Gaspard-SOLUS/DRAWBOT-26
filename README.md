# DRAWBOT-26
Projet DRAWBOT | Systèmes Bouclés | ECE 2026 | Groupe #S4S25-G-2314

# Drawbot - Projet Systèmes Bouclés

Projet réalisé dans le cadre du module Systèmes Bouclés.  
Objectif : commander un robot Drawbot avec ESP32, moteurs, encodeurs, IMU et magnétomètre.

## Fonctionnalités

- Interface web hébergée par l’ESP32
- Commande manuelle du robot : avancer, reculer, tourner, stop
- Affichage temps réel :
  - PWM moteurs
  - ticks encodeurs
  - distances roues
  - vitesses roues
  - données IMU
  - données magnétomètre
  - odométrie
- Console web de debug
- Envoi des données vers Teleplot en UDP

## Matériel

- NodeMCU ESP32
- Plateforme Gyrobot / Drawbot
- Deux moteurs DC avec encodeurs
- IMU LSM6DS3
- Magnétomètre LIS3MDL
- Driver moteurs DRV8837

## Compilation

Projet développé avec VS Code et PlatformIO.

SSID : DRAWBOT_#S4S25-G-2314
Mot de passe : !12345678!

Se connecter au résau puis ouvrir :
http://192.168.4.1

Commande de compilation :

```bash
pio run