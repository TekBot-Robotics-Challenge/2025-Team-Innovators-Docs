import { CodeViewer } from "../../components/CodeViewer";
import { Recycle, Zap, BarChart3, Globe, Target, Cpu, Palette, Monitor, Leaf, Trash2 } from 'lucide-react';

const TestFinal = () => {
    const projectStructure = `waste-sorting-system/
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   ├── WasteCard.jsx
│   │   │   └── Dashboard.jsx
│   │   ├── assets/
│   │   │   └── design.png
│   │   ├── App.jsx
│   │   └── main.jsx
│   ├── package.json
│   └── vite.config.js
├── backend/
│   ├── sensors/
│   │   └── color_detection.py
│   ├── api/
│   │   └── waste_data.js
│   └── conveyor/
│       └── sorting_logic.py
└── README.md`;

    const setupCommands = `# Cloner le projet
git clone https://github.com/innovators/waste-sorting-system.git
cd waste-sorting-system

# Installation des dépendances frontend
cd frontend
npm install

# Installation des dépendances backend
cd ../backend
pip install -r requirements.txt

# Lancement du système
npm run dev`;

    const technologiesUsed = `# Frontend
- React.js avec Vite
- Tailwind CSS pour le styling
- Lucide React pour les icônes

# Backend
- Node.js / Express.js
- Python pour les capteurs
- WebSocket pour les données temps réel

# Matériel
- Convoyeur intelligent
- Capteurs de couleur
- Système de tri automatique`;

    const wasteCategories = `const wasteTypes = {
  recyclable: {
    color: 'green',
    detection: ['blue', 'clear', 'white'],
    icon: 'recycle',
    percentage: 39.3
  },
  organic: {
    color: 'yellow', 
    detection: ['brown', 'green', 'orange'],
    icon: 'leaf',
    percentage: 28.5
  },
  electronic: {
    color: 'blue',
    detection: ['black', 'grey', 'metallic'],
    icon: 'zap',
    percentage: 13.5
  },
  residual: {
    color: 'red',
    detection: ['mixed', 'unknown'],
    icon: 'trash',
    percentage: 18.7
  }
};`;

    return (
        <div className="max-w-4xl mx-auto p-6 space-y-6">
            <h1 className="text-3xl font-bold text-green-600 flex items-center gap-2">
                <Globe className="w-6 h-6" /> Système de Tri Intelligent des Déchets
            </h1>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Target className="w-5 h-5" /> Projet : Interface Web Temps Réel
                </h2>
                <p className="text-gray-700">
                    Ce projet est une interface web en temps réel permettant de visualiser le tri des déchets détectés par un convoyeur intelligent développé par le groupe <strong>Innovators</strong>. Le système utilise la détection de couleur pour classifier automatiquement les déchets en 4 catégories principales.
                </p>
            </div>

            

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Zap className="w-5 h-5" /> Fonctionnalités
                </h2>
                <ul className="list-disc pl-6 text-gray-700 space-y-2">
                    <li><strong>📊 Affichage en temps réel</strong> du nombre total de déchets triés</li>
                    <li><strong>♻️ Classification automatique</strong> des déchets en 4 catégories :
                        <ul className="list-disc pl-6 mt-2 space-y-1">
                            <li><strong>Déchets recyclables</strong> (vert)</li>
                            <li><strong>Déchets organiques</strong> (jaune)</li>
                            <li><strong>Déchets électroniques</strong> (bleu)</li>
                            <li><strong>Déchets résiduels</strong> (rouge)</li>
                        </ul>
                    </li>
                    <li><strong>🌈 Tri basé sur la couleur</strong> détectée par le convoyeur</li>
                    <li><strong>📉 Calcul du pourcentage</strong> de chaque type de déchets</li>
                </ul>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Monitor className="w-5 h-5" /> Interface Utilisateur
                </h2>
                <p className="text-gray-700 mb-4">L'interface utilisateur affiche :</p>
                <ul className="list-disc pl-6 text-gray-700 space-y-2">
                    <li>Le <strong>nombre total de déchets triés</strong> au centre</li>
                    <li>Une <strong>carte pour chaque type de déchets</strong>, contenant :
                        <ul className="list-disc pl-6 mt-1 space-y-1">
                            <li>Le <strong>nombre d'unités</strong></li>
                            <li>Le <strong>pourcentage</strong> total</li>
                            <li>Une <strong>icône illustrant le type de déchet</strong></li>
                        </ul>
                    </li>
                    <li>Des <strong>couleurs spécifiques</strong> pour chaque catégorie pour une meilleure lisibilité</li>
                </ul>
            </div>

           

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <BarChart3 className="w-5 h-5" /> Statistiques Temps Réel
                </h2>
                <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                    <div className="bg-green-50 border-l-4 border-green-400 p-4 rounded">
                        <h3 className="font-semibold text-green-800 flex items-center gap-2">
                            <Recycle className="w-4 h-4" /> Recyclables
                        </h3>
                        <p className="text-2xl font-bold text-green-600">6,230</p>
                        <p className="text-sm text-green-700">39.3%</p>
                    </div>
                    <div className="bg-yellow-50 border-l-4 border-yellow-400 p-4 rounded">
                        <h3 className="font-semibold text-yellow-800 flex items-center gap-2">
                            <Leaf className="w-4 h-4" /> Organiques
                        </h3>
                        <p className="text-2xl font-bold text-yellow-600">4,512</p>
                        <p className="text-sm text-yellow-700">28.5%</p>
                    </div>
                    <div className="bg-blue-50 border-l-4 border-blue-400 p-4 rounded">
                        <h3 className="font-semibold text-blue-800 flex items-center gap-2">
                            <Zap className="w-4 h-4" /> Électroniques
                        </h3>
                        <p className="text-2xl font-bold text-blue-600">2,134</p>
                        <p className="text-sm text-blue-700">13.5%</p>
                    </div>
                    <div className="bg-red-50 border-l-4 border-red-400 p-4 rounded">
                        <h3 className="font-semibold text-red-800 flex items-center gap-2">
                            <Trash2 className="w-4 h-4" /> Résiduels
                        </h3>
                        <p className="text-2xl font-bold text-red-600">2,971</p>
                        <p className="text-sm text-red-700">18.7%</p>
                    </div>
                </div>
                <div className="mt-4 text-center">
                    <p className="text-3xl font-bold text-gray-800">Total trié : 15,847</p>
                </div>
            </div>

            <div className="bg-white rounded-lg p-6 shadow-md">
                <h2 className="text-2xl font-semibold mb-4 flex items-center gap-2">
                    <Target className="w-5 h-5" /> Objectif du Projet
                </h2>
                <div className="bg-green-50 border-l-4 border-green-400 p-4">
                    <p className="text-green-800">
                        Ce site fait partie d'un système global de tri intelligent, où un convoyeur détecte les déchets qui y passent en temps réel, les classe par type et couleur, puis transmet ces données à cette interface pour affichage.
                    </p>
                </div>
                <div className="mt-4 p-4 bg-gray-50 rounded-lg">
                    <p className="text-gray-700 italic">
                        <strong>Mission :</strong> Sensibiliser à la gestion intelligente des déchets à travers des outils visuels simples et efficaces.
                    </p>
                </div>
            </div>
        </div>
    );
};

export default TestFinal;