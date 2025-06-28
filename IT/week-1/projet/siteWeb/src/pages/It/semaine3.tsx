import { CodeViewer } from "../../components/CodeViewer";
import { Navigation, Route, Folder, Settings, Rocket, Brain, BarChart, Box, MapPin, Eye } from 'lucide-react';
import Gazebo from '../../assets/Gazebo.mp4';

const PathfindingDocumentation = () => {
    const projectStructure = `ros2_ws/
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
└── tekbot_sim/`;

    const setupCommands = `# Créer le workspace et copier le package
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

# Sourcer l'environnement
source install/setup.bash`;

    const launchCommands = `# Lancer Gazebo avec le robot et la carte
ros2 launch maze_solving tekbot_maze.launch.py gui:=true

# Démarrer le nœud de pathfinding (dans un nouveau terminal)
ros2 launch pathfinding_nav pathfinding_launch.py`;

    const rvizCommands = `source install/setup.bash
rviz2`;

    return (
        <div className="max-w-4xl mx-auto p-6 space-y-6">
            <h1 className="text-3xl font-bold text-green-600 flex items-center gap-2">
                <Navigation className="w-6 h-6" /> ROS 2 - Algorithme de Pathfinding (Semaine 3)
            </h1>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Route className="w-5 h-5" /> Projet : Navigation Autonome avec A*
                </h2>
                <p className="text-gray-700">
                    Ce projet présente la navigation autonome d'un robot dans un environnement simulé en utilisant <strong>Gazebo (interface graphique)</strong> et <strong>ROS2</strong>, avec l'algorithme de pathfinding <strong>A*</strong> et l'évitement d'obstacles via le LIDAR.
                </p>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Folder className="w-5 h-5" /> Structure du projet
                </h2>
                <CodeViewer 
                    code={projectStructure} 
                    language="bash" 
                    showLineNumbers={false}
                    className="my-4"
                />
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Settings className="w-5 h-5" /> Étapes de configuration
                </h2>
                <CodeViewer 
                    code={setupCommands} 
                    language="bash" 
                    className="my-4"
                />
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Rocket className="w-5 h-5" /> Lancer la simulation
                </h2>
                <CodeViewer 
                    code={launchCommands} 
                    language="bash" 
                    className="my-4"
                />
                <div className="bg-blue-50 border-l-4 border-blue-400 p-4 mt-4">
                    <p className="text-blue-800 font-medium">Ces commandes :</p>
                    <ul className="list-disc pl-6 text-blue-700 space-y-1 mt-2">
                        <li>Lance Gazebo avec le robot et la carte</li>
                        <li>Démarre le nœud de pathfinding basé sur A*</li>
                        <li>Le robot navigue de <code>(0, 0)</code> à <code>(9, 9)</code></li>
                    </ul>
                </div>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Brain className="w-5 h-5" /> Algorithme de Pathfinding – A*
                </h2>
                
                <h3 className="text-lg font-medium mb-2 text-green-600">Pourquoi utiliser A* ?</h3>
                <ul className="list-disc pl-6 mb-4 text-gray-700 space-y-1">
                    <li>Priorise les chemins les plus prometteurs</li>
                    <li>Plus rapide que Dijkstra</li>
                    <li>Utilise une heuristique (distance euclidienne)</li>
                </ul>

                <h3 className="text-lg font-medium mb-2 text-green-600">Fonctionnement</h3>
                <ul className="list-disc pl-6 mb-4 text-gray-700 space-y-1">
                    <li>L'environnement est modélisé en une grille 2D (10x10)</li>
                    <li>Cellules : <code>0</code> (libre), <code>1</code> (obstacle)</li>
                    <li>A* trouve le chemin optimal en utilisant un coût + une heuristique</li>
                    <li>Le robot suit les cellules via les messages <code>/cmd_vel</code></li>
                </ul>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <MapPin className="w-5 h-5" /> Évitement d'obstacles
                </h2>
                <ul className="list-disc pl-6 text-gray-700 space-y-2">
                    <li>Utilise le topic <code>/scan</code> (LaserScan)</li>
                    <li>Si un obstacle est détecté à moins de 0.5m :
                        <ul className="list-disc pl-6 mt-1 space-y-1">
                            <li>Le robot s'arrête</li>
                            <li>Il tourne jusqu'à libérer le passage</li>
                        </ul>
                    </li>
                </ul>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Eye className="w-5 h-5" /> Visualisation avec RViz2
                </h2>
                <CodeViewer 
                    code={rvizCommands} 
                    language="bash" 
                    className="my-4"
                />
                <div className="bg-gray-50 p-4 rounded-lg mt-4">
                    <p className="font-medium text-gray-800 mb-2">Ajoutez dans RViz2 :</p>
                    <ul className="list-disc pl-6 text-gray-700 space-y-1">
                        <li><code>LaserScan</code> → <code>/scan</code></li>
                        <li><code>Odometry</code> → <code>/odom</code></li>
                        <li><code>TF</code></li>
                        <li><code>RobotModel</code> ou <code>Grid</code></li>
                    </ul>
                </div>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <BarChart className="w-5 h-5" /> Résultat
                </h2>
                <div className="bg-green-50 border-l-4 border-green-400 p-4">
                    <ul className="list-disc pl-6 text-green-800 space-y-1">
                        <li>Le robot trouve le chemin optimal avec A*</li>
                        <li>Évite les obstacles en temps réel grâce au LIDAR</li>
                        <li>Le tout est visible sur Gazebo et RViz2</li>
                    </ul>
                </div>

                <div className="mt-6">
                    <video 
                        width="100%" 
                        className="rounded-lg shadow-md border"
                        autoPlay
                        loop
                    >
                        <source src={Gazebo} type="video/mp4" />
                        Votre navigateur ne supporte pas la lecture de vidéos.
                    </video>
                </div>
            </div>


            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Box className="w-5 h-5" /> Topics ROS2 utilisés
                </h2>
                <ul className="list-disc pl-6 text-gray-700 space-y-1">
                    <li><code>/cmd_vel</code> - Commandes de vitesse</li>
                    <li><code>/scan</code> - Données LIDAR</li>
                    <li><code>/odom</code> - Odométrie du robot</li>
                    <li><code>/tf</code> - Transformations spatiales</li>
                </ul>
            </div>
        </div>
    );
};

export default PathfindingDocumentation;