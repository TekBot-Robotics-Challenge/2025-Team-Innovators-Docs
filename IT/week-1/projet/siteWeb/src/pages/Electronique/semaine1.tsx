import { useState } from "react";
import { useCallback } from "react";
import { CodeViewer } from "../../../src/components/CodeViewer";
import accelrometre from "./images/accelerometre.jpg";
import DiagramSimulation from "./images/Diagramsimulation.jpg";
import MPU from "./images/MPU.jpg";
import MEMS from "./images/MEMS-Accelerometer-Working.gif";
import gyroscope from "./images/Coriolis-Force.png";

const ArduinoCode = `#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_MPU6050 mpu;

const float SEUIL = 1.5;    // m/s²
String lastDir = "";
float lastX=0, lastY=0, lastZ=0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    lcd.init(); lcd.backlight(); lcd.clear();
    if (!mpu.begin()) {
      lcd.print("Capteur KO !");
      while (1) delay(500);
      }
      lcd.setCursor((20-7)/2,1);
    lcd.print("BONJOUR"); delay(1000);
    calibrateMPU();  // Calibration au démarrage
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  String dir = detectDirection(accel);
  if (dir != lastDir) {
    lcd.setCursor(0,0); lcd.print("                ");
    lcd.setCursor(0,0); lcd.print(dir);
    lastDir = dir;
    }
    displayValues(accel);
    delay(100);
    }
    
    void calibrateMPU() {
      const int N = 200;
      float sumX=0, sumY=0, sumZ=0;
      for(int i=0; i<N; i++) {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        sumX += a.acceleration.x;
        sumY += a.acceleration.y;
        sumZ += a.acceleration.z - 9.806;
        delay(5);
        }
        float offX=sumX/N, offY=sumY/N, offZ=sumZ/N;
        Serial.printf("Offsets: X=%.2f Y=%.2f Z=%.2f\\n", offX, offY, offZ);
        }`;
const CapteurCode = `void calibrateMPU() {
            const int N = 200;
            float sumX=0, sumY=0, sumZ=0;
            for(int i=0; i<N; i++) {
                sensors_event_t a,g,t;
                mpu.getEvent(&a,&g,&t);
                sumX += a.acceleration.x;
                sumY += a.acceleration.y;
                sumZ += a.acceleration.z - 9.806;
                delay(5);
            }
            float offX=sumX/N, offY=sumY/N, offZ=sumZ/N;
            Serial.printf("Offsets: X=%.2f Y=%.2f Z=%.2f\n", offX, offY, offZ);
        }`;

const FiltreCode = `float angleRoll=0;
const float alpha=0.98;
String detectDirection(const sensors_event_t &a) {
    float rollAcc=atan2(a.acceleration.y,a.acceleration.z)*RAD_TO_DEG;
    float gyroRate=mpu.getGyroX()/131.0;
    angleRoll=alpha*(angleRoll+gyroRate*0.01f)+(1-alpha)*rollAcc;
    return String(angleRoll,1)+"°";
}`

const DmpCode = `#include <MPU6050_6Axis_MotionApps20.h>
MPU6050 mpu;  // Note : change d'API
Quaternion q;

void setup() {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
}

void loop() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.printf("Quaternion: w=%.3f x=%.3f y=%.3f z=%.3f\\n", q.w,q.x,q.y,q.z);
    }
}`
// Table des matières
export const TableOfContents = () => {
  const [isOpen, setIsOpen] = useState(false);
  return (
    <div className="bg-gray-50 border border-gray-200 rounded-lg p-4 mb-6">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-semibold text-gray-800 hover:text-blue-600"
      >
        Table des matières
        <span
          className={`transform transition-transform ${
            isOpen ? "rotate-180" : ""
          }`}
        >
          ▼
        </span>
      </button>
      {isOpen && (
        <div className="mt-4 space-y-1">
          <a
            href="#introduction"
            className="block text-blue-600 hover:underline"
          >
            1. Introduction
          </a>
          <a
            href="#principes-de-base"
            className="block text-blue-600 hover:underline"
          >
            2. Principes de base des capteurs
          </a>
          <div className="ml-4 space-y-1">
            <a
              href="#accelerometre"
              className="block text-blue-600 hover:underline"
            >
              2.1. Accéléromètre
            </a>
            <a
              href="#gyroscope"
              className="block text-blue-600 hover:underline"
            >
              2.2. Gyroscope
            </a>
          </div>
          <a
            href="#presentation-mpu6050"
            className="block text-blue-600 hover:underline"
          >
            3. Présentation du module MPU6050
          </a>
          <a
            href="#schema-cablage"
            className="block text-blue-600 hover:underline"
          >
            4. Schéma de câblage
          </a>
          <a
            href="#installation-bibliotheques"
            className="block text-blue-600 hover:underline"
          >
            5. Installation des bibliothèques
          </a>
          <a
            href="#exemple-code"
            className="block text-blue-600 hover:underline"
          >
            6. Exemple de code Arduino
          </a>
          <a href="#usage-dmp" className="block text-blue-600 hover:underline">
            7. Usage du DMP et quaternions
          </a>
          <a
            href="#simulation-wokwi"
            className="block text-blue-600 hover:underline"
          >
            8. Simulation Wokwi
          </a>
          <a href="#conclusion" className="block text-blue-600 hover:underline">
            9. Conclusion et perspectives
          </a>
        </div>
      )}
    </div>
  );
};

// Accéléromètre
export const AccelerometerConcept = () => (
  <div className="bg-gradient-to-br from-blue-50 to-indigo-100 border border-blue-200 rounded-lg p-6 my-6">
    <h4 className="font-semibold text-blue-800 mb-3">
      Principe de fonctionnement
    </h4>
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      <div className="bg-white p-4 rounded border">
        <div className="font-medium text-gray-800 mb-2">Au repos</div>
        <div className="text-sm text-gray-600">
          La bille repose sur la paroi Z (gravité = 1g)
        </div>
      </div>
      <div className="bg-white p-4 rounded border">
        <div className="font-medium text-gray-800 mb-2">En accélération</div>
        <div className="text-sm text-gray-600">
          La bille se déplace vers la paroi opposée
        </div>
      </div>
    </div>
  </div>
);

// Gyroscope
export const CoriolisExplanation = () => (
  <div className="bg-purple-50 border border-purple-200 rounded-lg p-6 my-6">
    <h4 className="font-semibold text-purple-800 mb-3">Effet Coriolis</h4>
    <p className="text-purple-700 mb-4">
      Deux masses vibrantes génèrent un déplacement perpendiculaire
      proportionnel à la vitesse angulaire.
    </p>
    <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
      <div className="bg-white p-3 rounded border text-center">
        <div className="font-medium text-red-600">Roll (X)</div>
        <div className="text-sm text-gray-600">Rotation autour de l'axe X</div>
      </div>
      <div className="bg-white p-3 rounded border text-center">
        <div className="font-medium text-green-600">Pitch (Y)</div>
        <div className="text-sm text-gray-600">Rotation autour de l'axe Y</div>
      </div>
      <div className="bg-white p-3 rounded border text-center">
        <div className="font-medium text-blue-600">Yaw (Z)</div>
        <div className="text-sm text-gray-600">Rotation autour de l'axe Z</div>
      </div>
    </div>
  </div>
);

// MPU6050 Specs
export const MPU6050Specs = () => (
  <div className="bg-gray-50 border border-gray-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-gray-800 mb-4">Spécifications techniques</h3>
    <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
      <div>
        <h4 className="font-semibold text-blue-600 mb-2">
          Caractéristiques générales
        </h4>
        <ul className="space-y-1 text-sm">
          <li>• Accéléromètre 3 axes + Gyroscope 3 axes</li>
          <li>• DMP (Digital Motion Processor) intégré</li>
          <li>• Alimentation : 3.3V (régulateur embarqué)</li>
          <li>• Compatible 5V sur I²C</li>
          <li>• Consommation : &lt; 3.6 mA (mesure), 5 µA (veille)</li>
        </ul>
      </div>
      <div>
        <h4 className="font-semibold text-green-600 mb-2">Plages de mesure</h4>
        <div className="space-y-2 text-sm">
          <div>
            <strong>Accéléromètre :</strong>
            <br />
            ±2g, ±4g, ±8g, ±16g
          </div>
          <div>
            <strong>Gyroscope :</strong>
            <br />
            ±250, ±500, ±1000, ±2000 °/s
          </div>
          <div>
            <strong>Température :</strong>
            <br />
            -40 à +85°C (±1°C)
          </div>
        </div>
      </div>
    </div>
  </div>
);

// Table de câblage
export const WiringTable = () => (
  <div className="overflow-x-auto my-6">
    <table className="min-w-full bg-white border border-gray-200 rounded-lg">
      <thead className="bg-gray-50">
        <tr>
          <th className="px-6 py-3 text-right text-xs font-medium text-gray-500 uppercase tracking-wider">
            Module MPU6050
          </th>
          <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
            Arduino UNO (R3)
          </th>
        </tr>
      </thead>
      <tbody className="divide-y divide-gray-200">
        <tr>
          <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 text-right">
            VCC
          </td>
          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
            5V
          </td>
        </tr>
        <tr className="bg-gray-50">
          <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 text-right">
            GND
          </td>
          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
            GND
          </td>
        </tr>
        <tr>
          <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 text-right">
            SDA
          </td>
          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
            A4 (SDA)
          </td>
        </tr>
        <tr className="bg-gray-50">
          <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 text-right">
            SCL
          </td>
          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
            A5 (SCL)
          </td>
        </tr>
        <tr>
          <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 text-right">
            AD0
          </td>
          <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
            non connecté (0x68) ou 3.3V pour 0x69
          </td>
        </tr>
      </tbody>
    </table>
  </div>
);

// Simulation Wokwi
export const WokwiSimulation = () => (
  <div className="bg-gradient-to-r from-green-50 to-blue-50 border border-green-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-green-800 mb-3">
      Testez en ligne sans matériel
    </h3>
    <p className="text-gray-700 mb-4">
      Utilisez la simulation Wokwi pour tester votre code avant de l'implémenter
      sur du matériel réel.
    </p>
    <a
      href="https://wokwi.com/projects/433364352341527553"
      target="_blank"
      rel="noopener noreferrer"
      className="inline-flex items-center px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors"
    >
      Ouvrir le projet Wokwi →
    </a>
  </div>
);

// Fichiers utiles

type FileItem = {
  name: string;
  file: string;
  type: "code" | "schematic" | "document" | "media";
  url: string; // Ajout d'une URL pour le téléchargement/consultation
};

export const FileLinks = () => {
  const files: FileItem[] = [
    {
      name: "Sketch Arduino",
      file: "sketch.ino",
      type: "code",
      url: "/documents/sketch.ino",
    },
    {
      name: "Schéma KiCad",
      file: "schema.kicad_sch",
      type: "schematic",
      url: "/documents/schema.kicad_sch",
    },
    {
      name: "PDF du circuit",
      file: "fdv.pdf",
      type: "document",
      url: "/documents/fdv.pdf",
    },
    {
      name: "Simulation virtuelle",
      file: "simulation.gif",
      type: "media",
      url: "/documents/simulation.gif",
    },
  ];

  const handleFileOpen = useCallback((file: FileItem) => {
    // Stratégie d'ouverture selon le type de fichier
    if (file.type === "document" || file.type === "media") {
      // Ouverture dans un nouvel onglet pour les PDF et médias
      window.open(file.url, "_blank");
    } else {
      // Téléchargement pour les autres types
      const link = document.createElement("a");
      link.href = file.url;
      link.download = file.file;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }
  }, []);

  const getTypeClasses = (type: FileItem["type"]) => {
    const classes = {
      code: "bg-blue-100 text-blue-700",
      schematic: "bg-green-100 text-green-700",
      document: "bg-red-100 text-red-700",
      media: "bg-purple-100 text-purple-700",
    };
    return classes[type] || "bg-gray-100 text-gray-700";
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4 my-6">
      {files.map((item, index) => (
        <div
          key={index}
          onClick={() => handleFileOpen(item)}
          className={`
            bg-white border border-gray-200 rounded-lg p-4 
            hover:shadow-md transition-all cursor-pointer
            hover:border-blue-300 active:scale-[0.98]
          `}
        >
          <div className="flex items-center justify-between">
            <div className="flex items-center truncate">
              {/* Vos icônes existantes ici */}
              <div className="truncate">
                <div className="font-medium text-gray-800 truncate">
                  {item.name}
                </div>
                <div className="text-sm text-gray-500 truncate">
                  {item.file}
                </div>
              </div>
            </div>
            <div className="flex items-center ml-2">
              <span
                className={`px-2 py-1 rounded text-xs font-medium ${getTypeClasses(
                  item.type
                )}`}
              >
                {item.type}
              </span>
            </div>
          </div>
        </div>
      ))}
    </div>
  );
};

// Résumé
export const Summary = () => (
  <div className="bg-gradient-to-br from-blue-50 to-purple-50 border border-blue-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-blue-800 mb-4">Ce guide couvre :</h3>
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-6">
      <ul className="space-y-2">
        <li className="flex items-center text-blue-700">
          <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
          Principes MEMS (accéléro, gyro)
        </li>
        <li className="flex items-center text-blue-700">
          <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
          Spécifications du MPU6050
        </li>
      </ul>
      <ul className="space-y-2">
        <li className="flex items-center text-blue-700">
          <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
          Câblage et code Arduino
        </li>
        <li className="flex items-center text-blue-700">
          <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
          Calibration et fusion de capteurs
        </li>
      </ul>
    </div>
    <h4 className="font-semibold text-purple-800 mb-2">À approfondir :</h4>
    <ul className="space-y-1 text-purple-700">
      <li>• Extraction de quaternions pour la réalité virtuelle</li>
      <li>• Compensation de la température en temps réel</li>
      <li>• Intégration d'un magnétomètre pour 9 DOF</li>
    </ul>
  </div>
);

// Ressources utiles
export const ResourceLinks = () => {
  const resources = [
    {
      title: "MPU6050 & Arduino",
      url: "https://randomnerdtutorials.com/arduino-mpu-6050-accelerometer-gyroscope/",
    },
    {
      title: "Tutoriel OLED & Arduino",
      url: "https://randomnerdtutorials.com/arduino-oled-display-ssd1306/",
    },
    {
      title: "Introduction aux capteurs MEMS",
      url: "https://www.electronics-tutorials.ws/io/io_7.html",
    },
    { title: "Documentation Arduino", url: "https://docs.arduino.cc/" },
    {
      title: "Bibliothèques Open Source",
      url: "https://github.com/Seeed-Studio/Seeed_Arduino_mpu6050",
    },
    {
      title: "Principes du DMP et quaternions",
      url: "https://www.invensense.com/wp-content/uploads/2015/02/AN-BNO080-01.pdf",
    },
    {
      title: "Projets DIY et Instructables",
      url: "https://instructables.com/tag/type-id/category-technology/",
    },
    {
      title: "Guides de programmation embarquée",
      url: "https://www.geeksforgeeks.org/embedded-systems/",
    },
  ];
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-3 my-6">
      {resources.map((resource, index) => (
        <a
          key={index}
          href={resource.url}
          target="_blank"
          rel="noopener noreferrer"
          className="flex items-center p-3 bg-white border border-gray-200 rounded-lg hover:shadow-md hover:border-blue-300 transition-all group"
        >
          <div className="w-8 h-8 bg-blue-100 rounded-full flex items-center justify-center mr-3 group-hover:bg-blue-200 transition-colors">
            <span className="text-blue-600 text-sm">🔗</span>
          </div>
          <div className="flex-1">
            <div className="font-medium text-gray-800 group-hover:text-blue-600 transition-colors">
              {resource.title}
            </div>
          </div>
          <span className="text-gray-400 group-hover:text-blue-400 transition-colors">
            →
          </span>
        </a>
      ))}
    </div>
  );
};

// Composant principal de la page
export default function Semaine1() {
  return (
    <div className="prose max-w-none">
      <TableOfContents />

      <h2
        id="introduction"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Introduction
      </h2>
      <p>
        La combinaison d'un accéléromètre et d'un gyroscope dans un seul
        composant permet de mesurer précisément l'orientation et le mouvement
        d'un objet. Le MPU6050, capteur 6-axes MEMS, intègre ces deux capteurs
        et un DMP (Digital Motion Processor) pour simplifier l'acquisition de
        données via I²C.
      </p>
      <div className="bg-blue-50 border-l-4 border-blue-400 p-4 my-4">
        <div className="font-semibold text-blue-800 mb-2">À retenir :</div>
        <ul className="text-blue-700 space-y-1">
          <li>
            Le MPU6050 mesure l'accélération (X, Y, Z) et la rotation (X, Y, Z)
          </li>
          <li>Interface I²C, deux adresses possibles (0x68 / 0x69)</li>
        </ul>
      </div>

      <h2
        id="principes-de-base"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Principes de base des capteurs
      </h2>
      <h3 id="accelerometre">Accéléromètre</h3>
      <p>
        Un accéléromètre mesure l'accélération linéaire. Voici le modèle
        conceptuel :
      </p>
      <div className="bg-gray-100 p-4 rounded-lg my-4">
        <div className="text-center text-gray-600 italic">
          Modèle simplifié : bille dans un cube en apesanteur
        </div>
        <div className="mt-3 flex justify-center">
          <img
            src={accelrometre}
            alt="Schéma simplifié d'une bille en apesanteur dans un cube"
            className="max-w-xs rounded shadow"
          />
        </div>
      </div>
      <ul>
        <li>
          En accélérant le cube de 1 g sur l'axe X, la bille appuie sur la paroi
          X
        </li>
        <li>
          Au repos sur Terre, la bille exerce 1 g sur la paroi Z, car la gravité
          agit en permanence
        </li>
      </ul>
      <AccelerometerConcept />
      <div className="bg-green-50 border-l-4 border-green-400 p-4 my-4">
        <div className="font-semibold text-green-800 mb-2">À retenir :</div>
        <ul className="text-green-700 space-y-1">
          <li>
            La lecture d'un accéléromètre inclut l'accélération due à la gravité
          </li>
          <li>
            Les capteurs MEMS utilisent la variation de capacité entre plaques
            pour mesurer la déflexion
          </li>
        </ul>
      </div>
      <h4>MEMS : fonctionnement interne</h4>
      <div className="mt-3 flex justify-center">
        <img
          src={MEMS}
          alt="animation montrant la structure suspendue par ressorts et la variation de capacité."
          className="max-w-xs rounded shadow"
        />
      </div>
      <p>
        Le capteur MEMS comporte un micro-masse montée sur des ressorts
        polysiliciés. Lors d'une accélération, la micro-masse se déplace,
        modifiant la distance entre plaques fixes et mobiles. Cette variation de
        capacité est proportionnelle à l'accélération.
      </p>

      <h3
        id="gyroscope"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Gyroscope
      </h3>
      <p>Un gyroscope mesure la rotation angulaire via l'effet Coriolis.</p>
      <div className="mt-3 flex justify-center">
        <img
          src={gyroscope}
          alt="schéma illustrant la force de Coriolis sur une masse en mouvement oscillatoire."
          className="max-w-xs rounded shadow"
        />
      </div>
      <CoriolisExplanation />
      <div className="bg-yellow-50 border-l-4 border-yellow-400 p-4 my-4">
        <div className="font-semibold text-yellow-800 mb-2">À retenir :</div>
        <p className="text-yellow-700">
          Les capteurs MEMS détectent la rotation par changement de capacité
          causé par l'effet Coriolis.
        </p>
      </div>

      <h2
        id="presentation-mpu6050"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Présentation du module MPU6050
      </h2>
      <div className="flex justify-center my-6">
        <img
          src={MPU}
          alt="vue du module MPU6050 avec régulateur et LED de puissance."
          className="max-w-md rounded shadow"
        />
      </div>
      <MPU6050Specs />
      <h4>Mesure de température</h4>
      <p>
        Le capteur embarque un thermomètre (-40 à +85°C, ±1°C). Il mesure la
        température de la puce pour compenser la dérive des capteurs, <b>pas</b>{" "}
        la température ambiante.
      </p>
      <h4>Bus I²C externe et 9 DOF</h4>
      <p>
        Deux broches (XDA, XCL) forment un bus I²C secondaire pour connecter un
        magnétomètre (ex : HMC5883L). Ainsi, on passe de 6 DOF à 9 DOF en
        mesurant le champ magnétique (X, Y, Z).
      </p>

      <h2 id="schema-cablage">Schéma de câblage Arduino ↔ MPU6050</h2>
      <WiringTable />
      <div className="flex justify-center my-6">
        <img
          src={DiagramSimulation}
          alt="Schéma de simulation du câblage MPU6050"
          className="max-w-md rounded shadow"
        />
      </div>

      <h2
        id="installation-bibliotheques"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Installation des bibliothèques
      </h2>
      <p>Installez via l'Arduino Library Manager :</p>
      <pre className="bg-gray-100 rounded p-4 overflow-x-auto">
        <code>
          Adafruit MPU6050 by Adafruit (≥ 1.1.0) Adafruit Unified Sensor by
          Adafruit (≥ 1.1.2) LiquidCrystal I2C by Frank de Brabander (≥ 1.1.4)
        </code>
      </pre>

      <h2 id="exemple-code">Exemple de code Arduino</h2>

      <CodeViewer
        code={ArduinoCode}
        language="cpp" // Utilisez "cpp" pour C++/Arduino
        className="my-4 border-2 border-blue-900/20"
      />

      <h4>Calibration du capteur</h4>
      <CodeViewer
        code={CapteurCode}
        language="cpp" // Utilisez "cpp" pour C++/Arduino
        className="my-4 border-2 border-blue-900/20"
      />

      <div className="bg-orange-50 border-l-4 border-orange-400 p-4 my-4">
        <div className="font-semibold text-orange-800">À retenir :</div>
        <p className="text-orange-700 mt-1">La calibration réduit les biais.</p>
      </div>

      <h4>Filtre Complémentaire (fusion accéléro/gyro)</h4>
      <CodeViewer
        code={FiltreCode}
        language="cpp" // Utilisez "cpp" pour C++/Arduino
        className="my-4 border-2 border-blue-900/20"
      />
      <div className="bg-purple-50 border-l-4 border-purple-400 p-4 my-4">
        <div className="font-semibold text-purple-800">À retenir :</div>
        <p className="text-purple-700 mt-1">
          Combine rapidité du gyro et stabilité de l'accéléro.
        </p>
      </div>

      <h2
        id="usage-dmp"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Usage du DMP et quaternions
      </h2>
      <p>
        Le DMP interne du MPU6050 calcule directement quaternions et vecteurs
        d'orientation, déchargeant l'Arduino :
      </p>
      <CodeViewer
        code={DmpCode}
        language="cpp" // Utilisez "cpp" pour C++/Arduino
        className="my-4 border-2 border-blue-900/20"
      />
     
      <div className="bg-red-50 border-l-4 border-red-400 p-4 my-4">
        <div className="font-semibold text-red-800">À retenir :</div>
        <p className="text-red-700 mt-1">
          Les quaternions évitent les problèmes de gimbal lock.
        </p>
      </div>

      <h2
        id="simulation-wokwi"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Simulation Wokwi
      </h2>
      <WokwiSimulation />

      <h2
        id="fichiers-utiles"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Fichiers utiles
      </h2>
      <FileLinks />

      <h2
        id="conclusion"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Conclusion et perspectives
      </h2>
      <Summary />

      <h2
        id="ressources-utiles"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Ressources utiles
      </h2>
      <ResourceLinks />
    </div>
  );
}
