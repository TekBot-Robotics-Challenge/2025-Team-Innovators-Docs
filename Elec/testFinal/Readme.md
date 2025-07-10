# 📘 Documentation Électronique du Système de Convoyeur

## Table des matières

1. [Études préliminaires](#1-études-préliminaires)
2. [Liste des pièces (BOM)](#2-liste-des-pièces-bom)
3. [Choix et justification des composants](#3-choix-et-justification-des-composants)
4. [Conception du schéma électronique](#4-conception-du-schéma-électronique)
5. [Schéma de câblage détaillé](#5-schéma-de-cablage-détaillé)
6. [Modélisation 3D et validation du PCB](#6-modélisation-3d-et-validation-du-pcb)
7. [Réalisation du PCB et câblage](#7-réalisation-du-pcb-et-cablage)
8. [Développement du code embarqué](#8-développement-du-code-embarqué)
9. [Plan de tests et validation](#9-plan-de-tests-et-validation)
10. [Gestion des erreurs et sécurité](#10-gestion-des-erreurs-et-sécurité)
11. [Plan de maintenance et évolutivité](#11-plan-de-maintenance-et-évolutivité)
12. [Structure de la documentation et présentation](#12-structure-de-la-documentation-et-présentation)
13. [Section vidéo](#13-section-vidéo)
14. [Journal des modifications](#14-journal-des-modifications)
15. [Annexes et références](#15-annexes-et-références)
16. [Conclusion](#16-conclusion)

---

## 1. Études préliminaires

Cette section établit les bases du projet en définissant clairement les besoins, contraintes et objectifs pour orienter la conception.

### 1.1 Objectifs et portée

- **Triage automatique** : Séparer quatre types de déchets par couleur (vert, jaune, rouge, bleu).
- **Interface web** : Suivi en temps réel des quantités triées.
- **Modularité** : Facilité d'adaptation pour le challenge final.

### 1.2 Exigences techniques

- **Détection couleur** : Précision ≥ 95 %, cubes de 30 mm.
- **Commande tapis** : Activation seulement en présence d'objet.
- **Communication** : JSON sur liaison série (ou Wifi selon choix).

### 1.3 Contraintes matériel

- **Dimensions mécaniques** : Longueur 650 mm, hauteur 100 mm.
- **Électronique** : Arduino Nano (ATmega328P), L298N, LiPo 7.4 V.
- **Sécurité** : Protection contre surintensité et surtensions.

---

## 2. Liste des pièces (BOM)

| Réf.  | Désignation                  | Qté | Fournisseur      | Prix (€) |
| ----- | ---------------------------- | --- | ---------------- | -------- |
| U1    | Arduino Nano (ATmega328P)    | 1   | Arduino Official | 22.00    |
| U2    | Capteur couleur TCS3200      | 1   | SparkFun         | 10.00    |
| U3    | Laser KY-008 + récepteur     | 1   | Amazon           | 4.50     |
| R1    | LDR 5 mm                     | 1   | Mouser           | 0.30     |
| R2    | Résistance 10 kΩ             | 1   | Mouser           | 0.10     |
| U4    | L298N Driver moteur          | 1   | eBay             | 5.00     |
| M1    | Moteur DC 12 V, 1.5 A max    | 1   | Pololu           | 12.00    |
| BT1   | Batterie LiPo 7.4 V 2200 mAh | 1   | HobbyKing        | 8.00     |
| SW1   | Interrupteur ON/OFF          | 1   | Farnell          | 1.20     |
| C1/C2 | Condensateurs 100 nF         | 2   | Mouser           | 0.05     |
| FUS1  | Fusible 2 A                  | 1   | RS Components    | 0.50     |
| –     | Connecteurs & câbles         | –   | –                | 3.00     |

> **Total estimé** : ≈ 66.65 €

---

## 3. Choix et justification des composants

Chaque composant est sélectionné pour répondre aux contraintes de coût, performance et fiabilité.

- **Arduino Nano** : microcontrôleur éprouvé, petite taille.
- **TCS3200** : capteur RGB digital, interface simple.
- **KY-008 + LDR** : double détection de présence pour robustesse.
- **L298N** : pilote moteur DC jusqu’à 2 A.
- **Batterie LiPo** : densité énergétique élevée.
- **Protection** : fusible pour surintensité, diodes pour éviter les pics.

---

## 4. Conception du schéma électronique

Cette étape traduit les besoins fonctionnels en schéma. On regroupe logique et puissance, ajoute protections et découplages.

1. **Broches Arduino** :
   - D2–D5 : S0–S3 du TCS3200
   - D6 : OUT du TCS3200
   - D7 : commande laser (via transistor)
   - A0 : signal LDR
   - D9–D11 : ENA, IN1, IN2 du L298N
2. **Protections** :
   - Condensateurs 100 nF près de chaque aliment.
   - Diodes de roue libre sur sorties moteur.
   - Fusible 2 A sur l’alimentation générale.
3. **Organisation** : séparer plan de masse logique et puissance, utiliser des poursuites larges pour le courant.



> **Astuce** : matérialiser les rails d’alimentation en polygon zones pour faciliter la fabrication.

---

## 5. Schéma de câblage détaillé

```text
+BT1 → SW1 → VIN Arduino & VCC L298N
GND BT1 → GND commun
D2 → TCS3200 S0
D3 → TCS3200 S1
D4 → TCS3200 S2
D5 → TCS3200 S3
D6 ← TCS3200 OUT
D7 → Transistor → KY-008 VCC
A0 ← Diviseur (R2/LDR)
D9 → ENA L298N
D10 → IN1 L298N
D11 → IN2 L298N
L298N OUT1/OUT2 → Moteur DC
```

> Chaque fil est étiqueté et gainé pour faciliter le diagnostic en cas de problème.

---

## 6. Modélisation 3D et validation du PCB

- **Export STEP** depuis KiCad.
- **Import** dans SolidWorks avec le convoyeur.
- **Vérification** : alignement, interférences, dégagement des câbles.

> Livrable : capture d’écran 3D annotée montrant l’intégration mécanique.

---

## 7. Réalisation du PCB et câblage

1. **Fabrication** : génération des Gerbers, contrôle DRC.
2. **Assemblage** : soudure à l’étain, contrôle optique des soudures.
3. **Tests avant mise sous tension** : continuité, absence de courts-circuits.
4. **Câblage final** : routage propre, gaines thermorétractables, étiquettes.

---

## 8. Développement du code embarqué

Le code se découpe en modules : détection, tri, communication.

### 8.1 Firmware (`convoyeur.ino`)

```cpp
#include <Arduino.h>
#include <ArduinoJson.h>

// Définition des broches
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define OUT 6

#define DETECT_PIN A0

#define ENA 9
#define IN1 10
#define IN2 11

const int seuilPresence = 300;
const uint32_t dureeTri = 1000; // Durée du tri en ms

uint16_t compteurVert = 0, compteurJaune = 0, compteurRouge = 0, compteurBleu = 0;

void setup() {
  Serial.begin(9600);
  // Initialisation des capteurs couleur
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  // Initialisation de la détection
  pinMode(DETECT_PIN, INPUT);
  // Initialisation du pilote moteur
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  stopperTapis();
}

void loop() {
  if (detectPresence()) {
    demarrerTapis();
    delay(200);  // Temps pour amener l'objet sous le capteur
    uint8_t couleur = lireCouleur();
    stopperTapis();
    tri(couleur);
    envoyerWeb();
    delay(500); // Anti-rebond
  }
}

bool detectPresence() {
  return analogRead(DETECT_PIN) < seuilPresence;
}

uint8_t lireCouleur() {
  uint32_t counts[4];
  const uint8_t filtres[4][2] = {{LOW,LOW},{HIGH,HIGH},{LOW,HIGH},{HIGH,LOW}};
  for (int i = 0; i < 4; i++) {
    digitalWrite(S2, filtres[i][0]);
    digitalWrite(S3, filtres[i][1]);
    delay(100);
    counts[i] = pulseIn(OUT, LOW, 50000);
  }
  uint8_t idx = 0;
  for (uint8_t i = 1; i < 4; i++) if (counts[i] < counts[idx]) idx = i;
  return idx;
}

void demarrerTapis() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
}

void stopperTapis() {
  analogWrite(ENA, 0);
}

void tri(uint8_t c) {
  switch (c) {
    case 0: compteurRouge++; break;
    case 1: compteurVert++; break;
    case 2: compteurBleu++; break;
    case 3: compteurJaune++; break;
  }
  delay(dureeTri);
}

void envoyerWeb() {
  StaticJsonDocument<200> doc;
  doc["rouge"] = compteurRouge;
  doc["vert"]  = compteurVert;
  doc["bleu"]  = compteurBleu;
  doc["jaune"] = compteurJaune;
  serializeJson(doc, Serial);
  Serial.println();
}
```

> **Explication** :
>
> - Les macros `#define` sont corrigées pour ne pas inclure de `;` et pointer vers les bonnes broches.
> - `detectPresence()` lit la photorésistance pour décider d'activer le tapis uniquement quand un objet est présent.
> - `lireCouleur()` sélectionne successivement les filtres du TCS3200, mesure les impulsions ( `pulseIn`) et retourne l'indice de la couleur dominante.
> - Les fonctions `demarrerTapis()` et `stopperTapis()` isolent la gestion du moteur pour plus de lisibilité.
> - `tri()` met à jour le compteur associé et attend `dureeTri` avant de stopper, assurant le bon positionnement.
> - `envoyerWeb()` utilise `ArduinoJson` pour sérialiser les compteurs en JSON et les envoyer via `Serial`.

> **Explication** :
>
> - Utilisation d’`ArduinoJson` pour sérialiser les compteurs.
> - Les fonctions `demarrerTapis()`, `lireCouleur()`, `tri()` et `envoyerWeb()` sont implémentées pour isoler la logique.

---

## 9. Plan de tests et validation

| Test                   | Méthode                        | Critère de réussite    |
| ---------------------- | ------------------------------ | ---------------------- |
| Détection présence     | Approche progressive (50→0 mm) | Signal fiable < seuil  |
| Reconnaissance couleur | 100 runs par couleur           | ≥ 95 % accuracy        |
| Moteur tapis           | Chronométrage activation       | < 200 ms               |
| Autonomie batterie     | Simulation charge continue     | ≥ 1 heure              |
| Communication web      | 1000 envois JSON               | < 1 % perte de paquets |

---

## 10. Gestion des erreurs et sécurité

- **Surintensité** : coupure via fusible.
- **Timeout capteur** : relance après 100 ms sans signal.
- **Validation JSON** : longueur minimale.
- **Protection EMI/EMC** : filtres RC sur alimentation moteur.

---

## 11. Plan de maintenance et évolutivité

- **Procedure de recalibration** : script Python pour seuils LDR/TCS3200.
- **Mises à jour** : support OTA (ESP8266 optionnel).
- **Modularité** : fichier de configuration JSON pour nouveaux capteurs.

---

## 12. Structure de la documentation et présentation

- **Clarté** : titres numérotés, sommaire auto.
- **Visuels** : captures d’écran, photos du PCB.
- **Présentation PPTX** : slides synthétiques avec logos TEKBOT & TRC 2025.

---

## 13. Section vidéo

🎬 **Démonstration** : [Voir la vidéo de test](https://github.com/mondepot/convoyeur/video.mp4)

> Chapitres : 00:00 Montage → 01:00 Tri en action → 02:00 Interface web

---

## 14. Journal des modifications

| Date       | Version | Description                                | Auteur      |
| ---------- | ------- | ------------------------------------------ | ----------- |
| 2025-07-10 | 1.0     | Documentation initiale                     | Équipe Élec |
| 2025-07-12 | 1.1     | Ajout code, tests, explications détaillées | Équipe Élec |

---

## 15. Annexes et références

- Datasheets : [TCS3200](URL), [L298N](URL).
- Normes CE/EMC utilizadas.
- Guides : KiCad, Arduino JSON.

---

## 16. Conclusion

Cette documentation exhaustive couvre toutes les étapes de conception, réalisation et test. 

```
```
