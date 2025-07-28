import { useState } from "react";
import { useCallback } from "react";
import { CodeViewer } from "../../../src/components/CodeViewer";
import { File, Code, Cpu as Circuit, FileText, Image, Play, Zap, Settings, Box, Cpu, Wrench } from "lucide-react";

export const TableOfContents = () => {
  const [isOpen, setIsOpen] = useState(false);
  return (
    <div className="bg-gradient-to-r from-blue-50 to-indigo-50 border-2 border-blue-200 rounded-xl p-6 mb-8 shadow-lg">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-bold text-gray-800 hover:text-blue-600 transition-colors duration-200"
      >
        <span className="text-lg">📚 Table des matières</span>
        <span className={`transform transition-transform duration-300 ${isOpen ? "rotate-180" : ""}`}>
          <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
          </svg>
        </span>
      </button>
      {isOpen && (
        <div className="mt-6 space-y-2 animate-fadeIn">
          <a href="#etudes-preliminaires" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            1. Études préliminaires
          </a>
          <a href="#liste-pieces" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            2. Liste des pièces (BOM)
          </a>
          <a href="#choix-composants" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            3. Choix et justification des composants
          </a>
          <a href="#schema-electronique" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            4. Conception du schéma électronique
          </a>
          <a href="#schema-cablage" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            5. Schéma de câblage détaillé
          </a>
          <a href="#modelisation-3d" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            6. Modélisation 3D et validation du PCB
          </a>
          <a href="#realisation-pcb" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            7. Réalisation du PCB et câblage
          </a>
          <a href="#code-embarque" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            8. Développement du code embarqué
          </a>
          <a href="#plan-tests" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            9. Plan de tests et validation
          </a>
          <a href="#gestion-erreurs" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            10. Gestion des erreurs et sécurité
          </a>
          <a href="#maintenance" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            11. Plan de maintenance et évolutivité
          </a>
          <a href="#structure-doc" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            12. Structure de la documentation et présentation
          </a>
          <a href="#section-video" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            13. Section vidéo
          </a>
          <a href="#journal-modifications" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            14. Journal des modifications
          </a>
          <a href="#annexes" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            15. Annexes et références
          </a>
          <a href="#conclusion" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            16. Conclusion
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
      name: "Firmware complet",
      file: "convoyeur.ino",
      type: "code",
      url: "#",
      description: "Code source Arduino pour le système de convoyeur"
    },
    {
      name: "Schémas électroniques",
      file: "schematics.zip",
      type: "schematic",
      url: "#",
      description: "Fichiers KiCad pour le PCB"
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
        <h3 className="text-2xl font-bold text-white">Démonstration du Système de Convoyeur</h3>
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
              <iframe 
                width="100%" 
                height="100%" 
                src="https://www.youtube.com/embed/VIDEO_ID" 
                title="YouTube video player" 
                frameBorder="0" 
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
                allowFullScreen
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
              <Settings className="w-5 h-5 text-blue-400" />
              <span className="text-white font-semibold">Technologie</span>
            </div>
            <p className="text-gray-300">Capteur couleur TCS3200 + Arduino Nano</p>
          </div>

          <div className="bg-gray-700 rounded-lg p-4">
            <div className="flex items-center gap-2 mb-2">
              <Zap className="w-5 h-5 text-yellow-400" />
              <span className="text-white font-semibold">Contrôle</span>
            </div>
            <p className="text-gray-300">L298N pour moteur DC</p>
          </div>

          <div className="bg-gray-700 rounded-lg p-4">
            <div className="flex items-center gap-2 mb-2">
              <Cpu className="w-5 h-5 text-green-400" />
              <span className="text-white font-semibold">Fonctionnalité</span>
            </div>
            <p className="text-gray-300">Tri automatique par couleur</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default function DocumentationConvoyeur() {
  return (
    <div className="prose max-w-none bg-gradient-to-br from-blue-50 to-indigo-50 min-h-screen">
      <div className="max-w-6xl mx-auto p-8">
        <TableOfContents />

        <div className="bg-white rounded-2xl shadow-xl p-8 mb-8">
          <h1 className="text-4xl font-bold bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent mb-8">
            📘 Documentation Électronique du Système de Convoyeur
          </h1>

          <h2 id="etudes-preliminaires" className="text-2xl font-bold text-blue-800 mt-8 mb-6 flex items-center gap-3">
            <Box className="w-8 h-8" />
            1. Études préliminaires
          </h2>
          <div className="bg-gradient-to-r from-blue-800 to-purple-800 text-white p-6 rounded-xl shadow-lg">
            <p className="text-lg leading-relaxed">
              Cette section établit les bases du projet en définissant clairement les besoins, contraintes et objectifs pour orienter la conception.
            </p>
          </div>

          <h3 className="text-xl font-bold text-blue-700 mt-6 mb-4">1.1 Objectifs et portée</h3>
          <ul className="list-disc pl-6 space-y-2 text-gray-700">
            <li><strong>Triage automatique</strong> : Séparer quatre types de déchets par couleur (vert, jaune, rouge, bleu).</li>
            <li><strong>Interface web</strong> : Suivi en temps réel des quantités triées.</li>
            <li><strong>Modularité</strong> : Facilité d'adaptation pour le challenge final.</li>
          </ul>

          <h3 className="text-xl font-bold text-blue-700 mt-6 mb-4">1.2 Exigences techniques</h3>
          <ul className="list-disc pl-6 space-y-2 text-gray-700">
            <li><strong>Détection couleur</strong> : Précision ≥ 95 %, cubes de 30 mm.</li>
            <li><strong>Commande tapis</strong> : Activation seulement en présence d'objet.</li>
            <li><strong>Communication</strong> : JSON sur liaison série (ou Wifi selon choix).</li>
          </ul>

          <h3 className="text-xl font-bold text-blue-700 mt-6 mb-4">1.3 Contraintes matériel</h3>
          <ul className="list-disc pl-6 space-y-2 text-gray-700">
            <li><strong>Dimensions mécaniques</strong> : Longueur 650 mm, hauteur 100 mm.</li>
            <li><strong>Électronique</strong> : Arduino Nano (ATmega328P), L298N, LiPo 7.4 V.</li>
            <li><strong>Sécurité</strong> : Protection contre surintensité et surtensions.</li>
          </ul>

          <h2 id="liste-pieces" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            2. Liste des pièces (BOM)
          </h2>

          <div className="overflow-hidden rounded-xl shadow-lg">
            <table className="min-w-full border-collapse">
              <thead>
                <tr className="bg-gradient-to-r from-blue-600 to-purple-600 text-white">
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Réf.</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Désignation</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Qté</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Fournisseur</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Prix (€)</th>
                </tr>
              </thead>
              <tbody className="bg-white">
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">U1</td>
                  <td className="border border-gray-300 px-6 py-4">Arduino Nano (ATmega328P)</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">Arduino Official</td>
                  <td className="border border-gray-300 px-6 py-4">22.00</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">U2</td>
                  <td className="border border-gray-300 px-6 py-4">Capteur couleur TCS3200</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">SparkFun</td>
                  <td className="border border-gray-300 px-6 py-4">10.00</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">U3</td>
                  <td className="border border-gray-300 px-6 py-4">Laser KY-008 + récepteur</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">Amazon</td>
                  <td className="border border-gray-300 px-6 py-4">4.50</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">R1</td>
                  <td className="border border-gray-300 px-6 py-4">LDR 5 mm</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">Mouser</td>
                  <td className="border border-gray-300 px-6 py-4">0.30</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">R2</td>
                  <td className="border border-gray-300 px-6 py-4">Résistance 10 kΩ</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">Mouser</td>
                  <td className="border border-gray-300 px-6 py-4">0.10</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">U4</td>
                  <td className="border border-gray-300 px-6 py-4">L298N Driver moteur</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">eBay</td>
                  <td className="border border-gray-300 px-6 py-4">5.00</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">M1</td>
                  <td className="border border-gray-300 px-6 py-4">Moteur DC 12 V, 1.5 A max</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">Pololu</td>
                  <td className="border border-gray-300 px-6 py-4">12.00</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">BT1</td>
                  <td className="border border-gray-300 px-6 py-4">Batterie LiPo 7.4 V 2200 mAh</td>
                  <td className="border border-gray-300 px-6 py-4">1</td>
                  <td className="border border-gray-300 px-6 py-4">HobbyKing</td>
                  <td className="border border-gray-300 px-6 py-4">8.00</td>
                </tr>
              </tbody>
              <tfoot className="bg-gray-100 font-bold">
                <tr>
                  <td colSpan={4} className="border border-gray-300 px-6 py-4 text-right">Total estimé</td>
                  <td className="border border-gray-300 px-6 py-4">≈ 66.65 €</td>
                </tr>
              </tfoot>
            </table>
          </div>

          <h2 id="choix-composants" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            3. Choix et justification des composants
          </h2>

          <div className="bg-blue-50 border-l-4 border-blue-400 p-6 rounded-lg">
            <p className="text-blue-700 leading-relaxed">
              Chaque composant est sélectionné pour répondre aux contraintes de coût, performance et fiabilité.
            </p>
            <ul className="list-disc pl-6 mt-4 space-y-2 text-blue-700">
              <li><strong>Arduino Nano</strong> : microcontrôleur éprouvé, petite taille.</li>
              <li><strong>TCS3200</strong> : capteur RGB digital, interface simple.</li>
              <li><strong>KY-008 + LDR</strong> : double détection de présence pour robustesse.</li>
              <li><strong>L298N</strong> : pilote moteur DC jusqu'à 2 A.</li>
              <li><strong>Batterie LiPo</strong> : densité énergétique élevée.</li>
              <li><strong>Protection</strong> : fusible pour surintensité, diodes pour éviter les pics.</li>
            </ul>
          </div>

          <h2 id="schema-electronique" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            4. Conception du schéma électronique
          </h2>

          <div className="bg-gradient-to-br from-indigo-50 to-purple-50 border-2 border-indigo-300 rounded-xl p-6">
            <p className="text-indigo-700">
              Cette étape traduit les besoins fonctionnels en schéma. On regroupe logique et puissance, ajoute protections et découplages.
            </p>
            
            <h3 className="text-xl font-bold text-indigo-700 mt-6 mb-4">Broches Arduino</h3>
            <ul className="list-disc pl-6 space-y-2 text-indigo-700">
              <li>D2–D5 : S0–S3 du TCS3200</li>
              <li>D6 : OUT du TCS3200</li>
              <li>D7 : commande laser (via transistor)</li>
              <li>A0 : signal LDR</li>
              <li>D9–D11 : ENA, IN1, IN2 du L298N</li>
            </ul>

            <h3 className="text-xl font-bold text-indigo-700 mt-6 mb-4">Protections</h3>
            <ul className="list-disc pl-6 space-y-2 text-indigo-700">
              <li>Condensateurs 100 nF près de chaque aliment.</li>
              <li>Diodes de roue libre sur sorties moteur.</li>
              <li>Fusible 2 A sur l'alimentation générale.</li>
            </ul>

            <h3 className="text-xl font-bold text-indigo-700 mt-6 mb-4">Organisation</h3>
            <p className="text-indigo-700">
              Séparer plan de masse logique et puissance, utiliser des poursuites larges pour le courant.
            </p>
          </div>

          <div className="mt-6 bg-yellow-50 border-l-4 border-yellow-400 p-6 rounded-lg">
            <p className="text-yellow-700 font-bold">
              Astuce : matérialiser les rails d'alimentation en polygon zones pour faciliter la fabrication.
            </p>
          </div>

          <h2 id="schema-cablage" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            5. Schéma de câblage détaillé
          </h2>

          <div className="bg-gray-800 rounded-xl p-6">
            <CodeViewer
              code={`+BT1 → SW1 → VIN Arduino & VCC L298N
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
L298N OUT1/OUT2 → Moteur DC`}
              language="text"
            />
          </div>

          <div className="mt-4 bg-blue-50 border-l-4 border-blue-400 p-6 rounded-lg">
            <p className="text-blue-700">
              Chaque fil est étiqueté et gainé pour faciliter le diagnostic en cas de problème.
            </p>
          </div>

          <h2 id="modelisation-3d" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            6. Modélisation 3D et validation du PCB
          </h2>

          <div className="bg-gradient-to-br from-green-50 to-emerald-50 border-2 border-green-300 rounded-xl p-6">
            <ul className="list-disc pl-6 space-y-2 text-green-700">
              <li><strong>Export STEP</strong> depuis KiCad.</li>
              <li><strong>Import</strong> dans SolidWorks avec le convoyeur.</li>
              <li><strong>Vérification</strong> : alignement, interférences, dégagement des câbles.</li>
            </ul>
            <p className="mt-4 text-green-700 font-semibold">
              Livrable : capture d'écran 3D annotée montrant l'intégration mécanique.
            </p>
          </div>

          <h2 id="realisation-pcb" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            7. Réalisation du PCB et câblage
          </h2>

          <div className="bg-gradient-to-br from-orange-50 to-amber-50 border-2 border-orange-300 rounded-xl p-6">
            <ol className="list-decimal pl-6 space-y-3 text-orange-700">
              <li><strong>Fabrication</strong> : génération des Gerbers, contrôle DRC.</li>
              <li><strong>Assemblage</strong> : soudure à l'étain, contrôle optique des soudures.</li>
              <li><strong>Tests avant mise sous tension</strong> : continuité, absence de courts-circuits.</li>
              <li><strong>Câblage final</strong> : routage propre, gaines thermorétractables, étiquettes.</li>
            </ol>
          </div>

          <h2 id="code-embarque" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            8. Développement du code embarqué
          </h2>

          <p className="text-gray-700">
            Le code se découpe en modules : détection, tri, communication.
          </p>

          <h3 className="text-xl font-bold text-blue-700 mt-6 mb-4">8.1 Firmware (`convoyeur.ino`)</h3>

          <div className="bg-gray-800 rounded-xl p-6">
            <CodeViewer
              code={`#include <Arduino.h>
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
}`}
              language="cpp"
            />
          </div>

          <div className="mt-6 bg-blue-50 border-l-4 border-blue-400 p-6 rounded-lg">
            <p className="text-blue-700">
              <strong>Explication :</strong>
              <ul className="list-disc pl-6 mt-2 space-y-2">
                <li>Les macros `#define` sont corrigées pour ne pas inclure de `;` et pointer vers les bonnes broches.</li>
                <li>`detectPresence()` lit la photorésistance pour décider d'activer le tapis uniquement quand un objet est présent.</li>
                <li>`lireCouleur()` sélectionne successivement les filtres du TCS3200, mesure les impulsions (`pulseIn`) et retourne l'indice de la couleur dominante.</li>
                <li>Les fonctions `demarrerTapis()` et `stopperTapis()` isolent la gestion du moteur pour plus de lisibilité.</li>
                <li>`tri()` met à jour le compteur associé et attend `dureeTri` avant de stopper, assurant le bon positionnement.</li>
                <li>`envoyerWeb()` utilise `ArduinoJson` pour sérialiser les compteurs en JSON et les envoyer via `Serial`.</li>
              </ul>
            </p>
          </div>

          <h2 id="plan-tests" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            9. Plan de tests et validation
          </h2>

          <div className="overflow-hidden rounded-xl shadow-lg">
            <table className="min-w-full border-collapse">
              <thead>
                <tr className="bg-gradient-to-r from-blue-600 to-purple-600 text-white">
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Test</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Méthode</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Critère de réussite</th>
                </tr>
              </thead>
              <tbody className="bg-white">
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Détection présence</td>
                  <td className="border border-gray-300 px-6 py-4">Approche progressive (50→0 mm)</td>
                  <td className="border border-gray-300 px-6 py-4">Signal fiable &lt; seuil</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Reconnaissance couleur</td>
                  <td className="border border-gray-300 px-6 py-4">100 runs par couleur</td>
                  <td className="border border-gray-300 px-6 py-4">≥ 95 % accuracy</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Moteur tapis</td>
                  <td className="border border-gray-300 px-6 py-4">Chronométrage activation</td>
                  <td className="border border-gray-300 px-6 py-4">&lt; 200 ms</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Autonomie batterie</td>
                  <td className="border border-gray-300 px-6 py-4">Simulation charge continue</td>
                  <td className="border border-gray-300 px-6 py-4">≥ 1 heure</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">Communication web</td>
                  <td className="border border-gray-300 px-6 py-4">1000 envois JSON</td>
                  <td className="border border-gray-300 px-6 py-4">&lt; 1 % perte de paquets</td>
                </tr>
              </tbody>
            </table>
          </div>

          <h2 id="gestion-erreurs" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            10. Gestion des erreurs et sécurité
          </h2>

          <div className="bg-gradient-to-br from-red-50 to-pink-50 border-2 border-red-300 rounded-xl p-6">
            <ul className="list-disc pl-6 space-y-2 text-red-700">
              <li><strong>Surintensité</strong> : coupure via fusible.</li>
              <li><strong>Timeout capteur</strong> : relance après 100 ms sans signal.</li>
              <li><strong>Validation JSON</strong> : longueur minimale.</li>
              <li><strong>Protection EMI/EMC</strong> : filtres RC sur alimentation moteur.</li>
            </ul>
          </div>

          <h2 id="maintenance" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            11. Plan de maintenance et évolutivité
          </h2>

          <div className="bg-gradient-to-br from-purple-50 to-violet-50 border-2 border-purple-300 rounded-xl p-6">
            <ul className="list-disc pl-6 space-y-2 text-purple-700">
              <li><strong>Procedure de recalibration</strong> : script Python pour seuils LDR/TCS3200.</li>
              <li><strong>Mises à jour</strong> : support OTA (ESP8266 optionnel).</li>
              <li><strong>Modularité</strong> : fichier de configuration JSON pour nouveaux capteurs.</li>
            </ul>
          </div>

          <h2 id="structure-doc" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            12. Structure de la documentation et présentation
          </h2>

          <div className="bg-gradient-to-br from-gray-50 to-blue-50 border-2 border-gray-300 rounded-xl p-6">
            <ul className="list-disc pl-6 space-y-2 text-gray-700">
              <li><strong>Clarté</strong> : titres numérotés, sommaire auto.</li>
              <li><strong>Visuels</strong> : captures d'écran, photos du PCB.</li>
              <li><strong>Présentation PPTX</strong> : slides synthétiques avec logos TEKBOT & TRC 2025.</li>
            </ul>
          </div>

          <h2 id="section-video" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            13. Section vidéo
          </h2>

          <VideoDemo />

          <div className="mt-4 bg-blue-50 border-l-4 border-blue-400 p-6 rounded-lg">
            <p className="text-blue-700">
              🎬 <strong>Démonstration</strong> : <a href="https://github.com/mondepot/convoyeur/video.mp4" className="text-blue-600 hover:underline">Voir la vidéo de test</a>
            </p>
            <p className="mt-2 text-blue-700">
              Chapitres : 00:00 Montage → 01:00 Tri en action → 02:00 Interface web
            </p>
          </div>

          <h2 id="journal-modifications" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            14. Journal des modifications
          </h2>

          <div className="overflow-hidden rounded-xl shadow-lg">
            <table className="min-w-full border-collapse">
              <thead>
                <tr className="bg-gradient-to-r from-blue-600 to-purple-600 text-white">
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Date</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Version</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Description</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Auteur</th>
                </tr>
              </thead>
              <tbody className="bg-white">
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">2025-07-10</td>
                  <td className="border border-gray-300 px-6 py-4">1.0</td>
                  <td className="border border-gray-300 px-6 py-4">Documentation initiale</td>
                  <td className="border border-gray-300 px-6 py-4">Équipe Élec</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4">2025-07-12</td>
                  <td className="border border-gray-300 px-6 py-4">1.1</td>
                  <td className="border border-gray-300 px-6 py-4">Ajout code, tests, explications détaillées</td>
                  <td className="border border-gray-300 px-6 py-4">Équipe Élec</td>
                </tr>
              </tbody>
            </table>
          </div>

          <h2 id="annexes" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            15. Annexes et références
          </h2>

          <div className="bg-gradient-to-br from-gray-100 to-gray-200 border-2 border-gray-300 rounded-xl p-6">
            <ul className="list-disc pl-6 space-y-2 text-gray-700">
              <li>Datasheets : <a href="#" className="text-blue-600 hover:underline">TCS3200</a>, <a href="#" className="text-blue-600 hover:underline">L298N</a>.</li>
              <li>Normes CE/EMC utilizadas.</li>
              <li>Guides : KiCad, Arduino JSON.</li>
            </ul>
          </div>

          <h2 id="conclusion" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            16. Conclusion
          </h2>

          <div className="bg-gradient-to-br from-green-50 to-teal-50 border-2 border-green-300 rounded-xl p-6">
            <p className="text-green-700 leading-relaxed">
              Cette documentation exhaustive couvre toutes les étapes de conception, réalisation et test du système de convoyeur de tri automatique. Elle sert de référence pour la maintenance future et les évolutions potentielles du système.
            </p>
          </div>

          {/* <FileLinks /> */}
        </div>
      </div>
    </div>
  );
}