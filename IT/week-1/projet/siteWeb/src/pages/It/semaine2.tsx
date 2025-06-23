import { CodeViewer } from "../../components/CodeViewer";
import { Satellite, FlaskConical as Flask, Folder, Wrench, Rocket, Brain, BarChart, Box } from 'lucide-react';


const ROS2Documentation = () => {
    const projectStructure = `sensor_data_evaluation/
├── launch/
│   └── sensor_launch.py
├── sensor_data_evaluation/
│   ├── __init__.py
│   ├── publisher_node.py
│   └── subscriber_node.py
├── package.xml
├── setup.py
└── README.md`;

    const installationCommands = `mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Cloner ou créer le package
cd ~/ros2_ws
colcon build
source install/setup.bash`;

    const launchCommand = `ros2 launch sensor_data_evaluation sensor_launch.py`;

    const exampleOutput = `[INFO] Publishing: {"temperature": 27.5, "humidity": 58.2, "pressure": 1003.1}
[INFO] ✅ Données valides
[INFO] Publishing: {"temperature": 42.1, "humidity": 60.0, "pressure": 1001.0}
[WARN] ⚠️ Données hors plage : {'temperature': 42.1, 'humidity': 60.0, 'pressure': 1001.0}`;

    return (
        <div className="max-w-4xl mx-auto p-6 space-y-6">
            <h1 className="text-3xl font-bold text-blue-600 flex items-center gap-2">
                <Satellite className="w-6 h-6" /> ROS 2 - Test de Validation de Données Capteurs
            </h1>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Flask className="w-5 h-5" /> Projet : <code className="font-mono bg-gray-100 px-2 py-1 rounded">sensor_data_evaluation</code>
                </h2>
                <p className="text-gray-700">
                    Ce projet ROS 2 simule l'envoi de données de capteurs (température, humidité, pression) via un <strong>publisher</strong>, et leur vérification via un <strong>subscriber</strong>. Il est conçu pour valider si les données reçues sont dans des plages acceptables.
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
                    <Wrench className="w-5 h-5" /> Installation
                </h2>
                <h3 className="text-lg font-medium mb-2">1. Créer l'espace de travail</h3>
                <CodeViewer 
                    code={installationCommands} 
                    language="bash" 
                    className="my-4"
                />
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Rocket className="w-5 h-5" /> Lancement du projet
                </h2>
                <CodeViewer 
                    code={launchCommand} 
                    language="bash" 
                    className="my-4"
                />
                <p className="text-gray-700 mt-2 italic">
                    Assure-toi d'avoir sourcé le bon environnement :
                </p>
                <CodeViewer 
                    code="source ~/ros2_ws/install/setup.bash" 
                    language="bash" 
                    className="my-4"
                />
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Brain className="w-5 h-5" /> Fonctionnement
                </h2>
                
                <h3 className="text-lg font-medium mb-2">Publisher (<code>publisher_node.py</code>)</h3>
                <ul className="list-disc pl-6 mb-4 text-gray-700 space-y-1">
                    <li>Publie toutes les 0.5 secondes un message JSON contenant :</li>
                    <li className="ml-4">Température (°C) : 15 à 35</li>
                    <li className="ml-4">Humidité (%) : 30 à 70</li>
                    <li className="ml-4">Pression (hPa) : 950 à 1050</li>
                </ul>

                <h3 className="text-lg font-medium mb-2">Subscriber (<code>subscriber_node.py</code>)</h3>
                <ul className="list-disc pl-6 mb-4 text-gray-700 space-y-1">
                    <li>Reçoit les messages et affiche :</li>
                    <li className="ml-4">✅ <code>Données valides</code> si toutes les valeurs sont dans la plage</li>
                    <li className="ml-4">⚠️ Un avertissement si une valeur est hors plage</li>
                </ul>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <BarChart className="w-5 h-5" /> Exemple de sortie console
                </h2>
                <CodeViewer 
                    code={exampleOutput} 
                    language="console" 
                    className="my-4"
                />
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Box className="w-5 h-5" /> Dépendances principales
                </h2>
                <ul className="list-disc pl-6 text-gray-700 space-y-1">
                    <li><code>rclpy</code></li>
                    <li><code>std_msgs</code></li>
                    <li><code>launch</code></li>
                    <li><code>launch_ros</code></li>
                </ul>
            </div>
        </div>
    );
};

export default ROS2Documentation;