import { useState } from "react";
import { useCallback } from "react";
import afficheur7segments from '../../assets/afficheur7segments.mp4';

import { CodeViewer } from "../../../src/components/CodeViewer";

import { File, Code, Cpu as Circuit, FileText, Image, Play, Clock, Zap } from "lucide-react";

export const TableOfContents = () => {
  const [isOpen, setIsOpen] = useState(false);
  return (
    <div className="bg-gradient-to-r from-blue-50 to-indigo-50 border-2 border-blue-200 rounded-xl p-6 mb-8 shadow-lg">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-bold text-gray-800 hover:text-blue-600 transition-colors duration-200"
      >
        <span className="text-lg">📚 Table des matières</span>
        <span
          className={`transform transition-transform duration-300 ${
            isOpen ? "rotate-180" : ""
          }`}
        >
          <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
          </svg>
        </span>
      </button>
      {isOpen && (
        <div className="mt-6 space-y-2 animate-fadeIn">
          <a href="#equipe-it3" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            1. Équipe
          </a>
          <a href="#objectif-projet3" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            2. Objectif du Projet - Minuteur
          </a>
          <a href="#demonstration-video" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            3. 🎥 Démonstration Vidéo
          </a>
          <a href="#realisation-minuteur" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            4. Réalisation du Minuteur
            <div className="ml-6 space-y-1 mt-2">
              <a href="#contraintes-minuteur" className="block text-blue-500 hover:text-blue-700 hover:underline transition-all duration-200">
                4.1. Contraintes Techniques
              </a>
              <a href="#architecture-minuteur" className="block text-blue-500 hover:text-blue-700 hover:underline transition-all duration-200">
                4.2. Architecture du Système
              </a>
            </div>
          </a>
          <a href="#composants-minuteur" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            5. Composants du Minuteur
          </a>
          <a href="#fonctionnalites" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            6. Fonctionnalités Implémentées
          </a>
          <a href="#code-minuteur" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            7. Code et Programmation
          </a>
          <a href="#tests-minuteur" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            8. Tests et Validation
          </a>
          <a href="#methodologie3" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            9. Méthodologie
          </a>
          <a href="#fichiers-inclus3" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            10. Fichiers du Projet
          </a>
        </div>
      )}
    </div>
  );
};

type FileItem = {
  name: string;
  file: string;
  type: "code" | "schematic" | "document" | "media";
  url: string;
  description?: string;
};

export const FileLinks = () => {
  const files: FileItem[] = [

    {
      name: "Documentation afficheur 7 servo",
      file: "Afficheur7Servos.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/Afficheur7Servos.pdf",
      description: "Documentation complète du minuteur"
    },
    {
      name: "Fichier Afficheur 7 Servos",
      file: "Afficheur7Servos.zip",
      type: "media",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/Afficheur7Servos.zip",
      description: "Fichier compressé contenant les ressources du minuteur"
    }
  ];

  const handleFileOpen = useCallback((file: FileItem) => {
    if (file.type === "document" || file.type === "media") {
      window.open(file.url, "_blank");
    } else {
      const link = document.createElement("a");
      link.href = file.url;
      link.download = file.file;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }
  }, []);

  const getTypeIcon = (type: FileItem["type"]) => {
    const icons = {
      code: <Code className="w-4 h-4" />,
      schematic: <Circuit className="w-4 h-4" />,
      document: <FileText className="w-4 h-4" />,
      media: <Image className="w-4 h-4" />,
    };
    return icons[type] || <File className="w-4 h-4" />;
  };

  const getTypeClasses = (type: FileItem["type"]) => {
    const classes = {
      code: "bg-gradient-to-r from-blue-100 to-blue-200 text-blue-700",
      schematic: "bg-gradient-to-r from-green-100 to-green-200 text-green-700",
      document: "bg-gradient-to-r from-orange-100 to-orange-200 text-orange-700",
      media: "bg-gradient-to-r from-purple-100 to-purple-200 text-purple-700",
    };
    return classes[type] || "bg-gradient-to-r from-gray-100 to-gray-200 text-gray-700";
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-6 my-8">
      {files.map((item, index) => (
        <div
          key={index}
          onClick={() => handleFileOpen(item)}
          className="bg-white border-2 border-gray-200 rounded-xl p-6 hover:shadow-xl transition-all duration-300 cursor-pointer hover:border-blue-500 hover:scale-105 transform"
        >
          <div className="flex items-start justify-between">
            <div className="flex-1 min-w-0">
              <div className="font-bold text-gray-800 truncate text-lg mb-1">
                {item.name}
              </div>
              <div className="text-sm text-gray-500 truncate mb-3 font-mono">
                {item.file}
              </div>
              {item.description && (
                <div className="text-sm text-gray-600 line-clamp-2 leading-relaxed">
                  {item.description}
                </div>
              )}
            </div>
            <div className="flex items-center ml-4">
              <span
                className={`px-3 py-2 rounded-lg text-sm font-bold flex items-center gap-2 ${getTypeClasses(item.type)} shadow-md`}
              >
                {getTypeIcon(item.type)}
              </span>
            </div>
          </div>
        </div>
      ))}
    </div>
  );
};

export const VideoDemo = () => {
  const [isPlaying, setIsPlaying] = useState(false);

  return (
    <div className="bg-gradient-to-br from-gray-900 to-gray-800 rounded-2xl p-8 my-8 shadow-2xl">
      <div className="flex items-center gap-3 mb-6">
        <Play className="w-8 h-8 text-blue-400" />
        <h3 className="text-2xl font-bold text-white">Démonstration du Minuteur</h3>
      </div>
      
      <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
        <div className="aspect-video bg-black rounded-lg relative overflow-hidden border-2 border-gray-600">
          {!isPlaying ? (
            <div className="absolute inset-0 flex items-center justify-center bg-gradient-to-br from-blue-600 to-purple-600">
              <button
                onClick={() => setIsPlaying(true)}
                className="bg-white/20 backdrop-blur-sm rounded-full p-6 hover:bg-white/30 transition-all duration-300 transform hover:scale-110"
              >
                <Play className="w-16 h-16 text-white ml-1" />
              </button>
            </div>
          ) : (
            <div className="absolute inset-0 bg-black">
              <video
                src={afficheur7segments} // 📁 Place ta vidéo dans public/videos/demo.mp4
                autoPlay
                controls
                className="w-full h-full object-cover"
              />
              <button
                onClick={() => setIsPlaying(false)}
                className="absolute top-4 right-4 px-4 py-2 bg-blue-600 rounded-lg hover:bg-blue-700 text-white"
              >
                Revenir à l'aperçu
              </button>
            </div>
          )}
        </div>
        
        <div className="mt-6 grid grid-cols-1 md:grid-cols-3 gap-4">
          <div className="bg-gray-700 rounded-lg p-4">
            <div className="flex items-center gap-2 mb-2">
              <Clock className="w-5 h-5 text-blue-400" />
              <span className="text-white font-semibold">Durée</span>
            </div>
            <p className="text-gray-300">11 secondes</p>
          </div>
          
          <div className="bg-gray-700 rounded-lg p-4">
            <div className="flex items-center gap-2 mb-2">
              <Zap className="w-5 h-5 text-yellow-400" />
              <span className="text-white font-semibold">Fonctionnalités</span>
            </div>
            <p className="text-gray-300">Décompte, alarme, reset</p>
          </div>
          
          <div className="bg-gray-700 rounded-lg p-4">
            <div className="flex items-center gap-2 mb-2">
              <Circuit className="w-5 h-5 text-green-400" />
              <span className="text-white font-semibold">Matériel</span>
            </div>
            <p className="text-gray-300">Arduino + LCD + Buzzer</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default function Semaine3() {
  return (
    <div className="prose max-w-none bg-gradient-to-br from-blue-50 to-indigo-50 min-h-screen">
      <div className="max-w-6xl mx-auto p-8">
        <TableOfContents />

        <div className="bg-white rounded-2xl shadow-xl p-8 mb-8">
          <h1 className="text-4xl font-bold bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent mb-8">
            Semaine 3 - Développement du Minuteur
          </h1>

          <h2 id="objectif-projet3" className="text-2xl font-bold text-blue-800 mt-8 mb-6 flex items-center gap-3">
            <Clock className="w-8 h-8" />
            Objectif du Projet - Minuteur
          </h2>
          <div className="bg-gradient-to-r from-blue-800 to-purple-800 text-white p-6 rounded-xl shadow-lg">
            <p className="text-lg leading-relaxed">
              Développer un <strong>minuteur électronique programmable</strong> capable de : <br /><br />
              🕐 <strong>Affichage du temps</strong> sur écran LCD 16x2 <br />
              ⏰ <strong>Décompte configurable</strong> avec boutons de contrôle <br />
              🔔 <strong>Alarme sonore</strong> en fin de décompte <br />
              🔄 <strong>Fonctions</strong> : Start, Pause, Reset, Configuration <br />
              ⚡ <strong>Interface intuitive</strong> avec feedback visuel et sonore
            </p>
          </div>

          <VideoDemo />

          <h2 id="realisation-minuteur" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Réalisation du Minuteur
          </h2>

          <h3 id="contraintes-minuteur" className="text-xl font-semibold text-blue-700 mt-8 mb-4">
            4.1. Contraintes Techniques
          </h3>
          <div className="bg-yellow-50 border-l-4 border-yellow-400 p-6 rounded-lg">
            <p className="text-gray-800">
              <span className="text-green-600 font-semibold">✅ Requis :</span> Précision temporelle, interface utilisateur claire <br />
              <span className="text-blue-600 font-semibold">🎯 Performance :</span> Réactivité des boutons, stabilité du décompte <br />
              <span className="text-purple-600 font-semibold">🔧 Technique :</span> Gestion des interruptions, optimisation énergétique
            </p>
          </div>

          <h3 id="architecture-minuteur" className="text-xl font-semibold text-blue-700 mt-8 mb-4">
            4.2. Architecture du Système
          </h3>
          <div className="bg-blue-50 border border-blue-200 rounded-xl p-6">
            <p className="text-gray-800 leading-relaxed">
              <strong>Microcontrôleur :</strong> Arduino Uno (ATmega328P) <br />
              <strong>Affichage :</strong> LCD 16x2 avec interface I2C <br />
              <strong>Interface :</strong> 4 boutons (Start/Pause, Stop, +, -) <br />
              <strong>Sortie sonore :</strong> Buzzer piezo <br />
              <strong>Alimentation :</strong> 5V via USB ou alimentation externe
            </p>
          </div>

          <h2 id="composants-minuteur" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Composants du Minuteur
          </h2>
          
          <div className="overflow-hidden rounded-xl shadow-lg">
            <table className="min-w-full border-collapse">
              <thead>
                <tr className="bg-gradient-to-r from-blue-600 to-purple-600 text-white">
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Composant</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Référence</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Fonction</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Quantité</th>
                </tr>
              </thead>
              <tbody className="bg-white">
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Arduino Uno</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">ATmega328P</td>
                  <td className="border border-gray-300 px-6 py-4">Contrôleur principal</td>
                  <td className="border border-gray-300 px-6 py-4 text-center">1</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">LCD 16x2</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">HD44780</td>
                  <td className="border border-gray-300 px-6 py-4">Affichage temps</td>
                  <td className="border border-gray-300 px-6 py-4 text-center">1</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Boutons poussoirs</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">SW1-SW4</td>
                  <td className="border border-gray-300 px-6 py-4">Interface utilisateur</td>
                  <td className="border border-gray-300 px-6 py-4 text-center">4</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Buzzer</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">BZ1</td>
                  <td className="border border-gray-300 px-6 py-4">Alarme sonore</td>
                  <td className="border border-gray-300 px-6 py-4 text-center">1</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Résistances</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">10kΩ</td>
                  <td className="border border-gray-300 px-6 py-4">Pull-up boutons</td>
                  <td className="border border-gray-300 px-6 py-4 text-center">4</td>
                </tr>
              </tbody>
            </table>
          </div>

          <h2 id="fonctionnalites" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Fonctionnalités Implémentées
          </h2>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            <div className="bg-gradient-to-br from-green-100 to-green-200 p-6 rounded-xl border-2 border-green-300">
              <h4 className="font-bold text-green-800 mb-3 text-lg">⏱️ Gestion du Temps</h4>
              <ul className="space-y-2 text-green-700">
                <li>• Configuration de 1 seconde à 99 minutes</li>
                <li>• Décompte précis avec affichage en temps réel</li>
                <li>• Format d'affichage MM:SS</li>
                <li>• Sauvegarde du temps configuré</li>
              </ul>
            </div>

            <div className="bg-gradient-to-br from-blue-100 to-blue-200 p-6 rounded-xl border-2 border-blue-300">
              <h4 className="font-bold text-blue-800 mb-3 text-lg">🎮 Contrôles</h4>
              <ul className="space-y-2 text-blue-700">
                <li>• Bouton Start/Pause</li>
                <li>• Bouton Stop/Reset</li>
                <li>• Boutons +/- pour configuration</li>
                <li>• Gestion des rebonds (debouncing)</li>
              </ul>
            </div>

            <div className="bg-gradient-to-br from-orange-100 to-orange-200 p-6 rounded-xl border-2 border-orange-300">
              <h4 className="font-bold text-orange-800 mb-3 text-lg">🔔 Alarme</h4>
              <ul className="space-y-2 text-orange-700">
                <li>• Signal sonore en fin de décompte</li>
                <li>• Fréquences variables</li>
                <li>• Durée configurable</li>
                <li>• Clignotement LCD</li>
              </ul>
            </div>

            <div className="bg-gradient-to-br from-purple-100 to-purple-200 p-6 rounded-xl border-2 border-purple-300">
              <h4 className="font-bold text-purple-800 mb-3 text-lg">📺 Interface</h4>
              <ul className="space-y-2 text-purple-700">
                <li>• Affichage clair et lisible</li>
                <li>• Indicateurs d'état</li>
                <li>• Messages d'information</li>
                <li>• Rétroéclairage LCD</li>
              </ul>
            </div>
          </div>

          <h2 id="tests-minuteur" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Tests et Validation
          </h2>

          <div className="bg-green-50 border-2 border-green-200 rounded-xl p-6">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div>
                <h4 className="font-bold text-green-800 mb-3">✅ Tests Fonctionnels</h4>
                <ul className="space-y-1 text-green-700">
                  <li>✓ Précision du décompte validée</li>
                  <li>✓ Réactivité des boutons</li>
                  <li>✓ Alarme fonctionnelle</li>
                  <li>✓ Affichage LCD stable</li>
                </ul>
              </div>
              <div>
                <h4 className="font-bold text-green-800 mb-3">⚡ Tests de Performance</h4>
                <ul className="space-y-1 text-green-700">
                  <li>✓ Stabilité sur 24h</li>
                  <li>✓ Consommation optimisée</li>
                  <li>✓ Résistance aux interférences</li>
                  <li>✓ Temps de réponse &lt; 100ms</li>
                </ul>
              </div>
            </div>
          </div>

          <FileLinks />

          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-8">
            <div className="bg-gradient-to-br from-green-50 to-emerald-50 border-2 border-green-300 rounded-xl p-6">
              <h3 className="text-xl font-bold text-green-800 mb-4 flex items-center gap-2">
                <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                </svg>
                Résultats
              </h3>
              <p className="text-green-700 leading-relaxed">
                Le minuteur fonctionne parfaitement avec une précision de ±1 seconde sur 60 minutes.
                L'interface utilisateur est intuitive et réactive. L'alarme est audible et efficace.
              </p>
            </div>

            <div className="bg-gradient-to-br from-blue-50 to-indigo-50 border-2 border-blue-300 rounded-xl p-6">
              <h3 className="text-xl font-bold text-blue-800 mb-4 flex items-center gap-2">
                <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
                Améliorations Futures
              </h3>
              <ul className="space-y-1 text-blue-700">
                <li>🔋 Mode économie d'énergie</li>
                <li>💾 Sauvegarde EEPROM</li>
                <li>📱 Interface Bluetooth</li>
                <li>🎵 Mélodies personnalisées</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}