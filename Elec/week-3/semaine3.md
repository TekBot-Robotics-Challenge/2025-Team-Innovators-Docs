# Projet : Afficheur 7 Segments Mécanique à Servomoteurs

## Table des matières
1. [Introduction](#introduction)  
2. [Objectifs](#objectifs)  
3. [Matériel et empreintes KiCad](#matériel-et-empreintes-kicad)  
4. [Conception du schéma et vérifications](#conception-du-schéma-et-vérifications)  
5. [Routage et tests PCB](#routage-et-tests-pcb)  
6. [Visualisation 3D](#visualisation-3d)  
7. [Firmware Arduino (code complet)](#firmware-arduino-code-complet)  
8. [Assemblage mécanique](#assemblage-mécanique)  
9. [Tests finaux et vidéo de démonstration](#tests-finaux-et-vidéo-de-démonstration)  
10. [Structure du dépôt GitHub](#structure-du-dépôt-github)  
11. [Conclusion](#conclusion)  
12. [Annexes et photos](#annexes-et-photos)  

---

## Introduction

Ce projet réalise un **afficheur 7 segments** entièrement mécanique, piloté par 7 servomoteurs SG90 connectés à un **ATmega328P** via un **PCA9685**.  

## Objectifs

- Schéma électronique validé sous KiCad  
- PCB testé sans breadboard  
- Firmware non bloquant  
- Documentation riche avec photos, captures et code  

## Matériel et empreintes KiCad

| Ref.  | Composant                   | Empreinte KiCad                             |
|-------|-----------------------------|----------------------------------------------|
| U1    | ATmega328P‑P                | `Package_DIP:DIP-28_W7.62mm`                 |
| U2    | AMS1117‑5.0                 | `Regulator_SMD:AMS1117-5.0`                  |
| U3    | PCA9685BS                   | `PCA9685:PCA9685-SOIC24`                     |
| Y1    | Quartz 16 MHz               | `Crystal:Crystal_HC49-4H_Vertical`           |
| C1–C13| Condensateurs (22 pF…10 µF) | `Capacitor_SMD:C_0805_2012Metric`, radial    |
| R1–R4 | Résistances (10 kΩ…330 Ω)    | `Resistor_SMD:R_0805_2012Metric`             |
| J1    | ISP 6 broches               | `Connector_Phoenix_MC:PhoenixContact_MC_1,5_6-G-3.5_1x06` |
| J2–J8 | Servo header 3 broches      | `Connector_PinHeader_2.54mm:PinHeader_1x03`  |
| J9    | Batterie header 2 broches   | `Connector_PinHeader_2.54mm:PinHeader_1x02`  |
| F1    | Polyfuse 2 A                | `Fuse_PTC:PTC_1812L6036R`                    |
| D1    | LED témoin 3 mm             | `LED_THT:LED_D3.0mm`                         |
| SW1   | Bouton Reset 6 mm           | `Switch_THT:SW_PUSH_6mm`                     |

## Conception du schéma et vérifications

1. **Zones** : Alim (gauche), MCU/driver (centre), servos (droite).  
2. **ERC** : aucun conflit.  
3. **Capture ERC** :  
   ![Test ERC](C:/Users/Andassa/Desktop/2025-Team-Innovators-Docs/Elec/week-3/photos/test erc schemas.png)  

## Routage et tests PCB

- **Placement** : composants d’alim, MCU, servos.  
- **Pistes** : VCC ≥1,5 mm, signaux 0,5 mm.  
- **Photos du test PCB** :  
  ![Attente PCB](C:/Users/Andassa/Desktop/2025-Team-Innovators-Docs/Elec/week-3/photos/test pcb.png)  
  ![PCB réussi](C:/Users/Andassa/Desktop/2025-Team-Innovators-Docs/Elec/week-3/photos/test pcb reussi .png)  

## Visualisation 3D

![3D face](photos/visualisation_3d.png)  
![3D dos](photos/visualisation dos.png)  
![3D gauche](photos/visualisation3d_gauche.png)  

## Firmware Arduino (code complet)

```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const uint16_t servoMin = 150, servoMax = 600;
const uint8_t map7seg[10] = {
  0b0111111,0b0000110,0b1011011,0b1001111,
  0b1100110,0b1101101,0b1111101,0b0000111,
  0b1111111,0b1101111
};
uint8_t digit = 0, dir = 1;
unsigned long lastTime = 0, interval = 1000;

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
}

void afficher(uint8_t d) {
  for(uint8_t i = 0; i < 7; i++) {
    bool on = map7seg[d] & (1 << i);
    pwm.setPWM(i, 0, on ? servoMax : servoMin);
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= interval) {
    lastTime = now;
    afficher(digit);
    digit += dir;
    if (digit == 9 || digit == 0) dir = -dir;
  }
}
Assemblage mécanique
Imprimer/découper 7 segments (3 × 15 × 3 mm).

Peindre l’arrière en noir.

Fixer chaque SG90, coller segments.

Ajuster cales 0°/90°.

Tests finaux et vidéo de démonstration
Vérifier la séquence 0→9→0.

Mesurer la consommation (~1 A/servo).

Réaliser une vidéo (20 s total).
Conclusion
Un projet complet alliant électronique, mécanique et firmware optimisé, documenté jusqu’aux tests et visuels.

Annexes et photos
Référez-vous aux images et captures intégrées dans chaque section pour plus de détails.







Demander à ChatGPT
