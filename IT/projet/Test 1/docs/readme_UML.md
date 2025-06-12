# 📊 Diagramme UML - Architecture des Robots

Ce document décrit la structure des classes du système de gestion des robots, conformément aux principes de la programmation orientée objet (POO).

## Diagramme de Classes
![Diagramme UML des Robots](class_diagram.png)

## Structure Principale

### Classe `Robots` (Classe Parente)
Classe de base représentant un robot générique avec ses caractéristiques fondamentales.

**Attributs:**
- `__nom` (str) : Nom du robot (privé)
- `__couleur` (str) : Couleur du robot (privé)
- `__energie` (int) : Niveau d'énergie (privé)
- `__x`, `__y` (int) : Position (privé)

**Méthodes:**
- `move(x, y)` : Déplace le robot
- `recharger()` : Réinitialise l'énergie à 100%
- `show()` : Affiche l'état du robot
- `action_speciale()` : Méthode polymorphe (à redéfinir)
- `travailler()` : Méthode polymorphe de comportement principal

**Propriétés:**
- `energie` : Getter/setter avec validation
- `x`, `y` : Accès en lecture seule à la position

## Sous-Classes Spécialisées

### 1. `RobotDomestique`
Robot dédié aux tâches ménagères, héritant de `Robots`.

**Attributs ajoutés:**
- `__taches_menageres` : Liste des tâches possibles
- `__taches_effectuees` : Historique des tâches

**Méthodes spécifiques:**
- `nettoyer(zone)` : Nettoie une zone spécifique
- `ranger(zone)` : Range une zone spécifique
- Surcharge de `move()` avec message contextuel
- Implémentation spécifique de `action_speciale()`

### 2. `RobotIndustriel`
Robot conçu pour la manutention lourde, héritant de `Robots`.

**Attributs ajoutés:**
- `__capacite_charge` : Poids maximum transportable
- `__charge_actuelle` : Poids actuellement porté

**Méthodes spécifiques:**
- `soulever(poids)` : Soulève une charge
- `deposer()` : Dépose la charge actuelle
- Surcharge de `move()` adaptée au port de charge
- Implémentation spécifique de `action_speciale()`

## Relations entre Classes

```plantuml
Robots <|-- RobotDomestique
Robots <|-- RobotIndustriel