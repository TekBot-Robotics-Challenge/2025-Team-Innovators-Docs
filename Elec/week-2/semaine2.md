# 📦 Projet : Boîte Noire & Station de Contrôle — Tekbot Robotics Challenge  
> **Version** : 1.0  
> **Type** : Réalisation sur Veroboard basée sur PCB  

---

## 🧭 Table des matières

1. [🎯 Objectif du projet](#1-objectif-du-projet)  
2. [🛠 Réalisation physique](#2-réalisation-physique)  
3. [📐 Vue d’ensemble des circuits](#3-vue-densemble-des-circuits)  
4. [📋 Liste des composants utilisés](#4-liste-des-composants-utilisés)  
5. [🔌 Communication I2C](#5-communication-i2c)  
6. [🧰 Fichiers KiCad](#6-fichiers-kicad)  
7. [🧪 Tests et simulations](#7-tests-et-simulations)  
8. [🔮 Perspectives](#8-perspectives)  
9. [📸 Preuves et photos](#9-preuves-et-photos)  
10. [📁 Structure du projet](#10-structure-du-projet)  
11. [🧪 Méthodologie du projet](#11-méthodologie-du-projet)  
12. [✅ Conclusion](#12-✅-conclusion)

---

## 1. 🎯 Objectif du projet

Développer une solution de **boîte noire embarquée** (dans un cube physique), capable de :
- Lire la **vitesse** et la **position spatiale**
- Transmettre les données via **I2C**
- Afficher les informations sur un **écran LCD 16x2** connecté à une station de contrôle

---

## 2. 🛠 Réalisation physique

### 🔧 Contraintes
- ❌ Interdit : Arduino et breadboard  
- ✅ Autorisé : Veroboard, circuits imprimés, composants discrets

### 🔨 Choix retenu
- 📐 Schéma et PCB créés avec **KiCad**  
- 🔌 **Réalisation finale sur veroboard**  
- 🗂 Les fichiers PCB sont des **perspectives à venir**

---

## 3. 📐 Vue d’ensemble des circuits

### 🧊 3.1. Cube – Boîte Noire
- **Microcontrôleur** : ATmega328P  
- **Capteur** : MPU6050 (gyroscope + accéléromètre)  
- **Rôle** : Maître I2C  
- **Réalisation** : veroboard (guidée par le PCB KiCad)

### 🖥️ 3.2. Station de Contrôle
- **Microcontrôleur** : ATmega328P  
- **Écran** : LCD 16x2 (mode 4 bits)  
- **Interface** : Esclave I2C  
- **Potentiomètre** : réglage contraste LCD  
- **Réalisation** : veroboard

---

## 4. 📋 Liste des composants utilisés

| Réf.         | Composant                              | Rôle                                    |
|--------------|----------------------------------------|------------------------------------------|
| U1 / U2      | ATmega328P-PU                          | Contrôle logique                         |
| MPU6050      | InvenSense MPU-6050                    | Gyroscope + accéléromètre                |
| LCD16x2      | Écran alphanumérique 16x2              | Affichage                                |
| Y1           | Quartz 16 MHz                          | Oscillateur du microcontrôleur           |
| C1, C2       | Condensateurs 22pF                     | Stabilité du quartz                      |
| R1           | Résistance 220Ω                        | Rétroéclairage LCD                       |
| R2           | Résistance 10kΩ                        | Pull-up Reset                            |
| SW1          | Bouton poussoir                        | Reset manuel                             |
| RV1          | Potentiomètre 10k                      | Contraste LCD                            |
| J1           | Connecteur I2C                         | Liaison Cube ↔ Station                   |
| Veroboard    | Plaque à bandes                        | Réalisation manuelle                     |
| Fils Dupont  | Connexions I2C                         | Câble 4 fils : GND / VCC / SDA / SCL     |

---

## 5. 🔌 Communication I2C

| Signal | Cube (Maître) | Station (Esclave) |
|--------|---------------|-------------------|
| SDA    | PC4           | PC4               |
| SCL    | PC5           | PC5               |
| VCC    | 5V commun     | 5V commun         |
| GND    | Masse commune | Masse commune     |

- I2C en mode multi-esclaves possible  
- Le maître contrôle l’envoi des mesures  
- L’esclave met à jour l’écran LCD à chaque réception

---

## 6. 🧰 Fichiers KiCad

### 📦 Cube (Boîte Noire)
- `boite_black.kicad_sch` : schéma logique  
- `boite_black.kicad_pcb` : PCB routé avec zone GND  
- **Utilisé comme modèle pour veroboard**

### 🖥️ Station de Contrôle
- `station_controle.kicad_sch` : schéma I2C + LCD  
- `station_controle.kicad_pcb` : PCB propre  
- **Utilisé comme base de placement manuel**

---

## 7. 🧪 Tests et simulations

- ✅ Test de continuité sur veroboard  
- ✅ Lecture des valeurs MPU6050 → transmission I2C  
- ✅ Affichage LCD 16x2 correct (position, vitesse)  
- ✅ Schémas validés par DRC = 0 erreur  
- 🛠 Tests réalisés avec Arduino IDE (hors présentation finale)

---

## 8. 🔮 Perspectives

- 🖨 **Fabrication PCB** en vue de la version finale  
- 🧩 Intégration dans un cube transparent 7×7×7 cm  
- 📦 Présentation propre sans breadboard  
- 💡 Ajout futur de **EEPROM externe** pour enregistrement

---

## 9. 📸 Preuves et photos

> 📷 À insérer ici avant présentation

- [ ] Cube veroboard – vue du dessus  
- [ ] Station veroboard – vue écran LCD  
- [ ] Liaison I2C entre les deux circuits  
- [ ] Schémas et PCB en capture d’écran

---

## 10. 📁 Structure du projet

```bash
projet-boite-noire/
├── README.md
├── boite_black.kicad_sch
├── boite_black.kicad_pcb
├── station_controle.kicad_sch
├── station_controle.kicad_pcb
├── gerbers/          # Pour futur PCB
├── veroboard_photos/ # À ajouter
└── code/             # Optionnel : code Arduino maître/esclave
11. 🧪 Méthodologie du projet
🔹 1. Analyse du cahier des charges
Lecture complète des exigences techniques (boîte noire + station)

Identification des contraintes : pas d’Arduino, pas de breadboard

Définition de l’architecture : maître I2C (cube) ↔ esclave I2C (station LCD)

🔹 2. Conception des schémas (KiCad)
Création de deux projets séparés :

boite_black.kicad_sch pour le cube

station_controle.kicad_sch pour la station

Choix des composants adaptés aux contraintes d’intégration (cube 7×7 cm)

🔹 3. Attribution des empreintes
Sélection des empreintes THT (Through Hole) compatibles veroboard

Attribution logique dans l’éditeur schématique

🔹 4. Routage PCB (perspective future)
Organisation claire des composants dans l’éditeur PCB

Ajout d’un plan de masse GND

Vérification via DRC (Design Rule Check) → 0 erreur

🔹 5. Réalisation sur veroboard
Utilisation du placement PCB comme guide visuel

Report manuel sur plaque à bandes (veroboard)

Soudures, coupes de bandes et connexions I2C

🔹 6. Tests de fonctionnement
Chargement du code maître / esclave via Arduino IDE

Lecture des données MPU6050 sur Cube

Réception et affichage sur LCD via I2C

Vérification de la stabilité de l’alimentation

🔹 7. Préparation à la présentation
Photos des montages

Nettoyage des circuits

Documentation sur Markdown

12. ✅ Conclusion
Ce projet a permis :

De comprendre l’interfaçage I2C entre microcontrôleurs

De créer une solution de visualisation temps réel

De concevoir une carte sur PCB puis de la réaliser à la main

De présenter un circuit compact, soigné, sans Arduino ni breadboard

📌 Le montage final est 100% fonctionnel sur veroboard
📌 Les fichiers KiCad sont prêts pour une fabrication professionnelle


