# 📡 ROS 2 - Test de Validation de Données Capteurs

## 🧪 Projet : `sensor_data_evaluation`

Ce projet ROS 2 simule l’envoi de données de capteurs (température, humidité, pression) via un **publisher**, et leur vérification via un **subscriber**. Il est conçu pour valider si les données reçues sont dans des plages acceptables.

---

## 📂 Structure du projet

```
sensor_data_evaluation/
├── launch/
│   └── sensor_launch.py
├── sensor_data_evaluation/
│   ├── __init__.py
│   ├── publisher_node.py
│   └── subscriber_node.py
├── package.xml
├── setup.py
└── README.md
```

---

## 🔧 Installation

### 1. Créer l’espace de travail
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Cloner ou créer le package
Crée ou place le dossier `sensor_data_evaluation` ici.

### 3. Compiler le projet
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🚀 Lancement du projet

Lance les deux nodes avec :

```bash
ros2 launch sensor_data_evaluation sensor_launch.py
```

> Assure-toi d’avoir sourcé le bon environnement :
```bash
source ~/ros2_ws/install/setup.bash
```

---

## 🧠 Fonctionnement

### Publisher (`publisher_node.py`)
- Publie toutes les 0.5 secondes un message JSON contenant :
  - Température (°C) : 15 à 35
  - Humidité (%) : 30 à 70
  - Pression (hPa) : 950 à 1050

### Subscriber (`subscriber_node.py`)
- Reçoit les messages et affiche :
  - ✅ `Données valides` si toutes les valeurs sont dans la plage.
  - ⚠️ Un avertissement si une valeur est hors plage.

---

## 📊 Exemple de sortie console

```
[INFO] Publishing: {"temperature": 27.5, "humidity": 58.2, "pressure": 1003.1}
[INFO] ✅ Données valides
[INFO] Publishing: {"temperature": 42.1, "humidity": 60.0, "pressure": 1001.0}
[WARN] ⚠️ Données hors plage : {'temperature': 42.1, 'humidity': 60.0, 'pressure': 1001.0}
```

---

## 🧰 Dépendances principales

- `rclpy`
- `std_msgs`
- `launch`
- `launch_ros`

---
  