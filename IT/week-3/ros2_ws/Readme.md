# Algorithme de Pathfinding – Projet de la Semaine 3

Ce projet présente la navigation autonome d’un robot dans un environnement simulé en utilisant **Gazebo (interface graphique)** et **ROS2**, avec l’algorithme de pathfinding **A\*** et l’évitement d’obstacles via le LIDAR.

---

## Structure du projet

```
ros2_ws/
├── src/
│   └── pathfinding_nav/
│       ├── launch/
│       │   └── pathfinding_launch.py
│       ├── pathfinding_nav/
│       │   ├── __init__.py
│       │   └── pathfinder.py
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
│      
└── tekbot_sim/
```

---

## Étapes de configuration

```bash
# Créer le workspace et copier le package
cd ~/ros2_ws
ros2 pkg create --build-type ament_python pathfinding_nav

# Création du fichier pathfinding.py pour l'algorithme
cd ~/ros2_ws/src/pathfinding_nav/pathfinding_nav/
touch pathfinder.py

# Création du fichier de lancement 
cd ~/ros2_ws/src/pathfinding_nav/
mkdir launch
cd /launch
touch pathfinding_launch.py

# Compiler le workspace
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build

# Sourcer l’environnement
source install/setup.bash
```

---

## Lancer la simulation

```bash
ros2 ros2 launch maze_solving tekbot_maze.launch.py gui:=true
```

```bash
ros2 launch pathfinding_nav pathfinding_launch.py
```

Ces commande :
- Lance Gazebo avec le robot et la carte
- Démarre le nœud de pathfinding basé sur A*
- Le robot navigue de `(0, 0)` à `(9, 9)`

---

## Algorithme de Pathfinding – A\*

### Pourquoi utiliser A* ?
- Priorise les chemins les plus prometteurs
- Plus rapide que Dijkstra
- Utilise une heuristique (distance euclidienne)

### Fonctionnement
- L’environnement est modélisé en une grille 2D (10x10)
- Cellules : `0` (libre), `1` (obstacle)
- A* trouve le chemin optimal en utilisant un coût + une heuristique
- Le robot suit les cellules via les messages `/cmd_vel`

---

## Évitement d’obstacles

- Utilise le topic `/scan` (LaserScan)
- Si un obstacle est détecté à moins de 0.5m :
  - Le robot s'arrête
  - Il tourne jusqu'à libérer le passage

---

## Visualisation avec RViz2

```bash
source install/setup.bash
rviz2
```

Ajoutez dans RViz2 :
- `LaserScan` → `/scan`
- `Odometry` → `/odom`
- `TF`
- `RobotModel` ou `Grid`

---

## Résultat

- Le robot trouve le chemin optimal avec A*
- Évite les obstacles en temps réel grâce au LIDAR
- Le tout est visible sur Gazebo et RViz2

---