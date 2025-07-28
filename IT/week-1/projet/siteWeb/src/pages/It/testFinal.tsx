import { Link } from "react-router-dom";
import { Recycle, Zap, BarChart3, Globe, Target, Monitor, Leaf, Trash2, ChevronRight } from 'lucide-react';
import conveyorImage from './assets/illustration (1).png';
import conveyorImage2 from './assets/illustration (2).png';

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

            <div className="bg-white rounded-lg overflow-hidden shadow-md">
                <img
                    src={conveyorImage}
                    alt="Convoyeur intelligent de tri des déchets"
                    className="w-full h-auto object-cover"
                />
                <img
                    src={conveyorImage2}
                    alt="Convoyeur intelligent de tri des déchets"
                    className="w-full h-auto object-cover"
                />
                <div className="p-4">
                    <h3 className="font-semibold text-lg text-gray-800 mb-2">Dashboard</h3>
                </div>
            </div>

            <div className="mt-8 text-center">
                <Link
                    to="/convoyeur"
                    className="inline-flex items-center px-6 py-3 bg-green-600 text-white font-medium rounded-lg hover:bg-green-700 transition-colors"
                >
                    Accéder au tableau de bord du convoyeur
                    <ChevronRight className="w-5 h-5 ml-2" />
                </Link>
            </div>
        </div>
    );
};

export default TestFinal;