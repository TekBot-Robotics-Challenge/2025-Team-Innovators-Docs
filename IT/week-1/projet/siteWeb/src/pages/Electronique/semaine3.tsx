import { useState } from "react";
import { useCallback } from "react";
import afficheur7segments from '../../assets/afficheur7segments.mp4';
import testErcSchemas from './images/test_erc_schemas.png';
import testPcb from './images/test_pcb.png';
import testPcbReussi from './images/test_pcb_reussi.png';
import visualisation3d from './images/visualisation_3d.png';
import visualisationDos from './images/visualisation_dos.png';
import visualisationGauche from './images/visualisaton3d_gauche.png';
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
          <a href="#introduction" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            1. Introduction
          </a>
          <a href="#objectifs" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            2. Objectifs
          </a>
          <a href="#materiel" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            3. Matériel et empreintes KiCad
          </a>
          <a href="#conception" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            4. Conception du schéma
          </a>
          <a href="#routage" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            5. Routage et tests PCB
          </a>
          <a href="#visualisation" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            6. Visualisation 3D
          </a>
          <a href="#firmware" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            7. Firmware Arduino
          </a>
          <a href="#assemblage" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            8. Assemblage mécanique
          </a>
          <a href="#tests" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            9. Tests finaux
          </a>
          <a href="#structure" className="block text-blue-600 hover:text-blue-800 hover:underline transition-all duration-200 font-medium">
            10. Structure du dépôt
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
      name: "Documentation complète",
      file: "Afficheur7Servos.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/Afficheur7Servos.pdf",
      description: "Documentation technique du projet"
    },
    {
      name: "Fichiers du projet",
      file: "Afficheur7Servos.zip",
      type: "media",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/Afficheur7Servos.zip",
      description: "Archive contenant tous les fichiers sources"
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
        <h3 className="text-2xl font-bold text-white">Démonstration de l'Afficheur 7 Segments</h3>
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
                src={afficheur7segments}
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
              <Settings className="w-5 h-5 text-blue-400" />
              <span className="text-white font-semibold">Technologie</span>
            </div>
            <p className="text-gray-300">7 servomoteurs SG90</p>
          </div>

          <div className="bg-gray-700 rounded-lg p-4">
            <div className="flex items-center gap-2 mb-2">
              <Zap className="w-5 h-5 text-yellow-400" />
              <span className="text-white font-semibold">Contrôle</span>
            </div>
            <p className="text-gray-300">PCA9685 + ATmega328P</p>
          </div>

          <div className="bg-gray-700 rounded-lg p-4">
            <div className="flex items-center gap-2 mb-2">
              <Cpu className="w-5 h-5 text-green-400" />
              <span className="text-white font-semibold">Animation</span>
            </div>
            <p className="text-gray-300">0→9→0 fluide</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default function Afficheur7Segments() {
  return (
    <div className="prose max-w-none bg-gradient-to-br from-blue-50 to-indigo-50 min-h-screen">
      <div className="max-w-6xl mx-auto p-8">
        <TableOfContents />

        <div className="bg-white rounded-2xl shadow-xl p-8 mb-8">
          <h1 className="text-4xl font-bold bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent mb-8">
            Afficheur 7 Segments Mécanique à Servomoteurs
          </h1>

          <h2 id="introduction" className="text-2xl font-bold text-blue-800 mt-8 mb-6 flex items-center gap-3">
            <Box className="w-8 h-8" />
            Introduction
          </h2>
          <div className="bg-gradient-to-r from-blue-800 to-purple-800 text-white p-6 rounded-xl shadow-lg">
            <p className="text-lg leading-relaxed">
              Ce projet innovant propose un <strong>afficheur 7 segments</strong> où chaque segment est actionné mécaniquement par un servomoteur SG90.
              Le cœur du pilotage est un <strong>ATmega328P</strong> commandé en I²C via un <strong>PCA9685</strong>.
              <br /><br />
              🔩 <strong>Mécanique :</strong> Rotation des bras pour dévoiler/masquer les segments<br />
              ⚡ <strong>Électronique :</strong> Schéma robuste validé sous KiCad<br />
              💻 <strong>Firmware :</strong> Programmation sans delay() pour animation fluide
            </p>
          </div>

          <VideoDemo />

          <h2 id="objectifs" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Objectifs
          </h2>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            <div className="bg-gradient-to-br from-blue-100 to-blue-200 p-6 rounded-xl border-2 border-blue-300">
              <h4 className="font-bold text-blue-800 mb-3 text-lg">Électronique</h4>
              <ul className="space-y-2 text-blue-700">
                <li>• Schéma robuste sous KiCad</li>
                <li>• Validation ERC complète</li>
                <li>• PCB optimisé</li>
              </ul>
            </div>

            <div className="bg-gradient-to-br from-green-100 to-green-200 p-6 rounded-xl border-2 border-green-300">
              <h4 className="font-bold text-green-800 mb-3 text-lg">Mécanique</h4>
              <ul className="space-y-2 text-green-700">
                <li>• Système sans breadboard</li>
                <li>• Segments actionnés précisément</li>
                <li>• Support solide</li>
              </ul>
            </div>

            <div className="bg-gradient-to-br from-purple-100 to-purple-200 p-6 rounded-xl border-2 border-purple-300">
              <h4 className="font-bold text-purple-800 mb-3 text-lg">Firmware</h4>
              <ul className="space-y-2 text-purple-700">
                <li>• Programmation non bloquante</li>
                <li>• Animation fluide 0→9→0</li>
                <li>• Documentation complète</li>
              </ul>
            </div>
          </div>

          <h2 id="materiel" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Matériel et empreintes KiCad
          </h2>

          <div className="overflow-hidden rounded-xl shadow-lg">
            <table className="min-w-full border-collapse">
              <thead>
                <tr className="bg-gradient-to-r from-blue-600 to-purple-600 text-white">
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Réf.</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Composant</th>
                  <th className="border border-blue-500 px-6 py-4 text-left font-bold">Empreinte KiCad</th>
                </tr>
              </thead>
              <tbody className="bg-white">
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">U1</td>
                  <td className="border border-gray-300 px-6 py-4">ATmega328P-P</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">Package_DIP:DIP-28_W7.62mm</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">U2</td>
                  <td className="border border-gray-300 px-6 py-4">AMS1117-5.0</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">Regulator_SMD:AMS1117-5.0</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">U3</td>
                  <td className="border border-gray-300 px-6 py-4">PCA9685BS</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">PCA9685:PCA9685-SOIC24</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">J1-J8</td>
                  <td className="border border-gray-300 px-6 py-4">Connecteurs Servo</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">PinHeader_1x03_P2.54mm</td>
                </tr>
                <tr className="hover:bg-blue-50 transition-colors">
                  <td className="border border-gray-300 px-6 py-4 font-bold">Y1</td>
                  <td className="border border-gray-300 px-6 py-4">Quartz 16MHz</td>
                  <td className="border border-gray-300 px-6 py-4 font-mono">Crystal_HC49-4H_Vertical</td>
                </tr>
              </tbody>
            </table>
          </div>

          <h2 id="conception" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Conception du schéma
          </h2>

          <div className="bg-blue-50 border border-blue-200 rounded-xl p-6">
            <p className="text-gray-800 leading-relaxed">
              <strong>Disposition :</strong> Alimentation à gauche, MCU et driver au centre, servos à droite<br />
              <strong>Vérifications :</strong> ERC complet pour absence de conflits<br />
              <strong>Optimisation :</strong> Minimisation des longueurs de pistes critiques<br />
              <strong>Alimentation :</strong> LDO 5V avec filtrage capacitif
            </p>
          </div>

          <div className="my-6 bg-white p-4 rounded-xl border-2 border-gray-200">
            <img src={testErcSchemas} alt="Test ERC" className="rounded-lg shadow-md w-full" />
            <p className="text-center text-sm text-gray-500 mt-2">Vérification ERC du schéma sous KiCad</p>
          </div>

          <h2 id="routage" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Routage et tests PCB
          </h2>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            <div className="bg-gradient-to-br from-green-50 to-emerald-50 p-6 rounded-xl border-2 border-green-300">
              <h4 className="font-bold text-green-800 mb-3">Stratégie de routage</h4>
              <ul className="space-y-2 text-green-700">
                <li>• VCC : largeur ≥ 1,5mm (jusqu'à 3A)</li>
                <li>• GND : plan de masse continu</li>
                <li>• Signaux PWM : largeur 0,5mm</li>
                <li>• Groupement par fonction</li>
              </ul>
            </div>
            <div className="bg-gradient-to-br from-blue-50 to-indigo-50 p-6 rounded-xl border-2 border-blue-300">
              <h4 className="font-bold text-blue-800 mb-3">Tests PCB</h4>
              <ul className="space-y-2 text-blue-700">
                <li>• Vérification des courts-circuits</li>
                <li>• Test de continuité</li>
                <li>• Validation des alimentations</li>
                <li>• Test fonctionnel servomoteurs</li>
              </ul>
            </div>
          </div>

          <div className="my-6 grid grid-cols-1 md:grid-cols-2 gap-4">
            <div className="bg-white p-4 rounded-xl border-2 border-gray-200">
              <img src={testPcb} alt="PCB en test" className="rounded-lg shadow-md w-full" />
              <p className="text-center text-sm text-gray-500 mt-2">Montage en cours de soudure</p>
            </div>
            <div className="bg-white p-4 rounded-xl border-2 border-gray-200">
              <img src={testPcbReussi} alt="PCB final" className="rounded-lg shadow-md w-full" />
              <p className="text-center text-sm text-gray-500 mt-2">PCB final validé</p>
            </div>
          </div>

          <h2 id="visualisation" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Visualisation 3D
          </h2>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            <div className="bg-white p-4 rounded-xl border-2 border-gray-200">
              <img src={visualisation3d} alt="Vue 3D face" className="rounded-lg shadow-md w-full" />
              <p className="text-center text-sm text-gray-500 mt-2">Vue de face</p>
            </div>
            <div className="bg-white p-4 rounded-xl border-2 border-gray-200">
              <img src={visualisationDos} alt="Vue 3D dos" className="rounded-lg shadow-md w-full" />
              <p className="text-center text-sm text-gray-500 mt-2">Vue arrière</p>
            </div>
            <div className="bg-white p-4 rounded-xl border-2 border-gray-200">
              <img src={visualisationGauche} alt="Vue 3D côté" className="rounded-lg shadow-md w-full" />
              <p className="text-center text-sm text-gray-500 mt-2">Vue de côté</p>
            </div>
          </div>

          <h2 id="firmware" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Firmware Arduino
          </h2>

          <div className="bg-gray-800 rounded-xl p-6">
            <CodeViewer
              code={`#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint16_t servoMin = 150, servoMax = 600;

const uint8_t map7seg[10] = {
  0b0111111, 0b0000110, 0b1011011, 0b1001111,
  0b1100110, 0b1101101, 0b1111101, 0b0000111,
  0b1111111, 0b1101111
};

uint8_t digit = 0, dir = 1;
unsigned long lastTime = 0, interval = 1000;

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
}

void afficher(uint8_t d) {
  for (uint8_t i = 0; i < 7; i++) {
    bool on = (map7seg[d] >> i) & 0x1;
    pwm.setPWM(i, 0, on ? servoMax : servoMin);
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= interval) {
    lastTime = now;
    afficher(digit);
    digit += dir;
    if (digit == 9 || digit == 0) dir = -dir;
  }
}`}
              language="cpp"
            />
          </div>

          <div className="mt-6 bg-blue-50 border-l-4 border-blue-400 p-6 rounded-lg">
            <p className="text-blue-700">
              <strong>Fonctionnement :</strong> Le code utilise un tableau de bits pour mapper les chiffres 0-9 aux segments A-G.
              La fonction <code>afficher()</code> positionne chaque servo en fonction de l'état du segment correspondant.
              La temporisation est gérée sans <code>delay()</code> pour une animation fluide.
            </p>
          </div>

          <h2 id="assemblage" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Assemblage mécanique
          </h2>

          <div className="bg-gradient-to-br from-orange-50 to-amber-50 border-2 border-orange-300 rounded-xl p-6">
            <ul className="space-y-3 text-orange-700">
              <li>• <strong>Segments :</strong> Rectangles 3×15×3 mm (impression 3D ou découpe)</li>
              <li>• <strong>Finition :</strong> Peinture noire au dos pour masquer les segments repliés</li>
              <li>• <strong>Support :</strong> Fixation solide des SG90 (alu ou plexi)</li>
              <li>• <strong>Cales :</strong> Butées à 0° et 90° pour positionnement précis</li>
              <li>• <strong>Alignement :</strong> Vérification de la cohérence visuelle</li>
            </ul>
          </div>

          <h2 id="tests" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Tests finaux
          </h2>

          <div className="bg-green-50 border-2 border-green-200 rounded-xl p-6">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div>
                <h4 className="font-bold text-green-800 mb-3">Tests fonctionnels</h4>
                <ul className="space-y-2 text-green-700">
                  <li>✓ Séquence 0→9→0 fluide</li>
                  <li>✓ Temps de réponse des servos</li>
                  <li>✓ Alimentation stable sous charge</li>
                  <li>✓ Robustesse mécanique</li>
                </ul>
              </div>
              <div>
                <h4 className="font-bold text-green-800 mb-3">Mesures</h4>
                <ul className="space-y-2 text-green-700">
                  <li>• Consommation : ~1A max (7 servos)</li>
                  <li>• Précision angulaire : ±2°</li>
                  <li>• Durée de vie : 1000+ cycles</li>
                </ul>
              </div>
            </div>
          </div>

          <h2 id="structure" className="text-2xl font-bold text-blue-800 mt-12 mb-6">
            Structure du dépôt GitHub
          </h2>

          <div className="bg-gray-100 border border-gray-300 rounded-xl p-6 font-mono">
            <pre className="whitespace-pre overflow-x-auto">
              {`afficheur7servos/
├─ photos/
│  ├─ test_erc_schemas.png
│  ├─ test_pcb.png
│  ├─ test_pcb_reussi.png
│  ├─ visualisation_3d_face.png
│  ├─ visualisation_3d_dos.png
│  └─ visualisation_3d_left.png
├─ schematic/
├─ pcb/
├─ 3D/
├─ firmware/
│  └─ afficheur7seg.ino
├─ datasheets/
└─ README.md`}
            </pre>
          </div>

          <FileLinks />

          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-8">
            <div className="bg-gradient-to-br from-green-50 to-emerald-50 border-2 border-green-300 rounded-xl p-6">
              <h3 className="text-xl font-bold text-green-800 mb-4 flex items-center gap-2">
                <Wrench className="w-6 h-6" />
                Résultats
              </h3>
              <p className="text-green-700 leading-relaxed">
                L'afficheur mécanique fonctionne avec précision et fluidité.
                La documentation complète permet une reproduction aisée du projet.
                Le système est robuste et visuellement impressionnant.
              </p>
            </div>

            <div className="bg-gradient-to-br from-blue-50 to-indigo-50 border-2 border-blue-300 rounded-xl p-6">
              <h3 className="text-xl font-bold text-blue-800 mb-4 flex items-center gap-2">
                <Settings className="w-6 h-6" />
                Améliorations futures
              </h3>
              <ul className="space-y-1 text-blue-700">
                <li>• Affichage multi-digit</li>
                <li>• Contrôle WiFi/Bluetooth</li>
                <li>• Boîtier professionnel</li>
                <li>• Optimisation énergétique</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}