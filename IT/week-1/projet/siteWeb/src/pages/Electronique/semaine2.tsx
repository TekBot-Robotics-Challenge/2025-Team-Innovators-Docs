import { useState } from "react";
import { useCallback } from "react";
import { CodeViewer } from "../../../src/components/CodeViewer";

import { File, Code, Cpu as Circuit, FileText, Image } from "lucide-react";

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
            href="#equipe-it2"
            className="block text-blue-600 hover:underline"
          >
            1. Équipe
          </a>
          <a
            href="#objectif-projet2"
            className="block text-blue-600 hover:underline"
          >
            2. Objectif du Projet
          </a>
          <a
            href="#realisation-physique"
            className="block text-blue-600 hover:underline"
          >
            3. Réalisation Physique
            <div className="ml-4 space-y-1">
              <a
                href="#contraintes"
                className="block text-blue-600 hover:underline"
              >
                3.1. Contraintes
              </a>
              <a
                href="#choix-retenu"
                className="block text-blue-600 hover:underline"
              >
                3.2. Choix Retenu
              </a>
            </div>
          </a>
          <a
            href="#vue-ensemble-circuits"
            className="block text-blue-600 hover:underline"
          >
            4. Vue d'Ensemble des Circuits
            <div className="ml-4 space-y-1">
              <a
                href="#cube-boite-noire"
                className="block text-blue-600 hover:underline"
              >
                4.1. Cube - Boîte Noire
              </a>
              <a
                href="#station-controle"
                className="block text-blue-600 hover:underline"
              >
                4.2. Station de Contrôle
              </a>
            </div>
          </a>
          <a
            href="#composants-utilises"
            className="block text-blue-600 hover:underline"
          >
            5. Liste des Composants
          </a>
          <a
            href="#communication-i2c"
            className="block text-blue-600 hover:underline"
          >
            6. Communication I2C
          </a>
          <a
            href="#fichiers-kicad"
            className="block text-blue-600 hover:underline"
          >
            7. Fichiers KiCad
          </a>
          <a
            href="#tests-simulations"
            className="block text-blue-600 hover:underline"
          >
            8. Tests et Simulations
          </a>
          <a
            href="#methodologie"
            className="block text-blue-600 hover:underline"
          >
            9. Méthodologie du Projet
          </a>
          <a
            href="#notes-equipe-it2"
            className="block text-blue-600 hover:underline"
          >
            10. Notes de l'Équipe
          </a>
          <a
            href="#etapes-completees2"
            className="block text-blue-600 hover:underline"
          >
            11. Étapes Complétées
          </a>
          <a
            href="#fichiers-inclus2"
            className="block text-blue-600 hover:underline"
          >
            12. Fichiers Inclus dans le Dépôt GitHub
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
      name: "Schéma Boîte Noire",
      file: "boite_black.kicad_sch",
      type: "schematic",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/boite_black.kicad_sch",
      description: "Schéma logique du cube avec ATmega328P et MPU6050"
    },
    {
      name: "PCB Boîte Noire",
      file: "boite_black.kicad_pcb",
      type: "schematic",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/boite_black.kicad_pcb",
      description: "PCB routé avec zone GND pour réalisation veroboard"
    },
    {
      name: "Schéma Station de Contrôle",
      file: "station_controle.kicad_sch",
      type: "schematic",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/station_controle.kicad_sch",
      description: "Schéma I2C + LCD 16x2"
    },
    {
      name: "PCB Station de Contrôle",
      file: "station_controle.kicad_pcb",
      type: "schematic",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/station_controle.kicad_pcb",
      description: "PCB propre pour base de placement manuel"
    },
   
    {
      name: "Documentation station de contrôle",
      file: "station_controle.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/station_controle.pdf",
      description: "Documentation technique complète de la station de contrôle"
    },

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
      code: "bg-blue-100 text-blue-700",
      schematic: "bg-green-100 text-green-700",
      document: "bg-orange-100 text-orange-700",
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
          className="bg-white border border-gray-200 rounded-lg p-4 hover:shadow-md transition-all cursor-pointer hover:border-blue-500 active:scale-[0.98]"
        >
          <div className="flex items-start justify-between">
            <div className="flex-1 min-w-0">
              <div className="font-medium text-gray-800 truncate">
                {item.name}
              </div>
              <div className="text-sm text-gray-500 truncate mb-2">
                {item.file}
              </div>
              {item.description && (
                <div className="text-xs text-gray-600 line-clamp-2">
                  {item.description}
                </div>
              )}
            </div>
            <div className="flex items-center ml-2">
              <span
                className={`px-2 py-1 rounded text-xs font-medium flex items-center gap-1 ${getTypeClasses(item.type)}`}
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

export default function Semaine2() {
  return (
    <div className="prose max-w-none">
      <TableOfContents />
   

      <h2 id="objectif-projet2" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Objectif du Projet
      </h2>
      <p className="box-border border-2 border-dashed p-4 bg-blue-800 text-white border-blue-800">
        Développer une solution de <b>boîte noire embarquée</b> (dans un cube physique), capable de : <br /><br />
        • Lire la <b>vitesse</b> et la <b>position spatiale</b> via capteur MPU6050 <br />
        • Transmettre les données via protocole <b>I2C</b> <br />
        • Afficher les informations sur un <b>écran LCD 16x2</b> connecté à une station de contrôle <br />
        • Respecter les contraintes : pas d'Arduino ni de breadboard en présentation finale
      </p>

      <h2 id="realisation-physique" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Réalisation Physique
      </h2>

      <h3 id="contraintes" className="text-lg font-semibold text-blue-700 mt-6 mb-3">
        3.1. Contraintes
      </h3>
      <p>
        <span className="text-red-600">❌ Interdit :</span> Arduino et breadboard en présentation finale <br />
        <span className="text-green-600">✅ Autorisé :</span> Veroboard, circuits imprimés, composants discrets <br />
        <span className="text-blue-600">📐 Dimension :</span> Cube de 7×7×7 cm maximum
      </p>

      <h3 id="choix-retenu" className="text-lg font-semibold text-blue-700 mt-6 mb-3">
        3.2. Choix Retenu
      </h3>
      <p>
        • Schéma et PCB créés avec <b>KiCad</b> <br />
        • <b>Réalisation finale sur veroboard</b> <br />
        • Les fichiers PCB sont des <b>perspectives futures</b> pour fabrication professionnelle <br />
        • Utilisation du placement PCB comme guide pour le montage manuel
      </p>

      <h2 id="vue-ensemble-circuits" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Vue d'Ensemble des Circuits
      </h2>

      <h3 id="cube-boite-noire" className="text-lg font-semibold text-blue-700 mt-6 mb-3">
        4.1. Cube – Boîte Noire
      </h3>
      <p>
        <b>Microcontrôleur :</b> ATmega328P <br />
        <b>Capteur :</b> MPU6050 (gyroscope + accéléromètre) <br />
        <b>Rôle :</b> Maître I2C <br />
        <b>Réalisation :</b> Veroboard (guidée par le PCB KiCad) <br />
        <b>Alimentation :</b> 5V via connecteur externe
      </p>

      <h3 id="station-controle" className="text-lg font-semibold text-blue-700 mt-6 mb-3">
        4.2. Station de Contrôle
      </h3>
      <p>
        <b>Microcontrôleur :</b> ATmega328P <br />
        <b>Écran :</b> LCD 16x2 (mode 4 bits) <br />
        <b>Interface :</b> Esclave I2C <br />
        <b>Potentiomètre :</b> Réglage contraste LCD <br />
        <b>Réalisation :</b> Veroboard avec composants THT
      </p>

      <h2 id="composants-utilises" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Liste des Composants Utilisés
      </h2>
      
      <div className="overflow-x-auto">
        <table className="min-w-full border-collapse border border-gray-300">
          <thead>
            <tr className="bg-gray-100">
              <th className="border border-gray-300 px-4 py-2 text-left">Référence</th>
              <th className="border border-gray-300 px-4 py-2 text-left">Composant</th>
              <th className="border border-gray-300 px-4 py-2 text-left">Rôle</th>
            </tr>
          </thead>
          <tbody>
            <tr><td className="border border-gray-300 px-4 py-2">U1 / U2</td><td className="border border-gray-300 px-4 py-2">ATmega328P-PU</td><td className="border border-gray-300 px-4 py-2">Contrôle logique</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">MPU6050</td><td className="border border-gray-300 px-4 py-2">InvenSense MPU-6050</td><td className="border border-gray-300 px-4 py-2">Gyroscope + accéléromètre</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">LCD16x2</td><td className="border border-gray-300 px-4 py-2">Écran alphanumérique 16x2</td><td className="border border-gray-300 px-4 py-2">Affichage des données</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">Y1</td><td className="border border-gray-300 px-4 py-2">Quartz 16 MHz</td><td className="border border-gray-300 px-4 py-2">Oscillateur du microcontrôleur</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">C1, C2</td><td className="border border-gray-300 px-4 py-2">Condensateurs 22pF</td><td className="border border-gray-300 px-4 py-2">Stabilité du quartz</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">R1</td><td className="border border-gray-300 px-4 py-2">Résistance 220Ω</td><td className="border border-gray-300 px-4 py-2">Rétroéclairage LCD</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">R2</td><td className="border border-gray-300 px-4 py-2">Résistance 10kΩ</td><td className="border border-gray-300 px-4 py-2">Pull-up Reset</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">SW1</td><td className="border border-gray-300 px-4 py-2">Bouton poussoir</td><td className="border border-gray-300 px-4 py-2">Reset manuel</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">RV1</td><td className="border border-gray-300 px-4 py-2">Potentiomètre 10k</td><td className="border border-gray-300 px-4 py-2">Contraste LCD</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">J1</td><td className="border border-gray-300 px-4 py-2">Connecteur I2C</td><td className="border border-gray-300 px-4 py-2">Liaison Cube ↔ Station</td></tr>
          </tbody>
        </table>
      </div>

      <h2 id="communication-i2c" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Communication I2C
      </h2>
      
      <div className="overflow-x-auto">
        <table className="min-w-full border-collapse border border-gray-300">
          <thead>
            <tr className="bg-gray-100">
              <th className="border border-gray-300 px-4 py-2 text-left">Signal</th>
              <th className="border border-gray-300 px-4 py-2 text-left">Cube (Maître)</th>
              <th className="border border-gray-300 px-4 py-2 text-left">Station (Esclave)</th>
            </tr>
          </thead>
          <tbody>
            <tr><td className="border border-gray-300 px-4 py-2">SDA</td><td className="border border-gray-300 px-4 py-2">PC4</td><td className="border border-gray-300 px-4 py-2">PC4</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">SCL</td><td className="border border-gray-300 px-4 py-2">PC5</td><td className="border border-gray-300 px-4 py-2">PC5</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">VCC</td><td className="border border-gray-300 px-4 py-2">5V commun</td><td className="border border-gray-300 px-4 py-2">5V commun</td></tr>
            <tr><td className="border border-gray-300 px-4 py-2">GND</td><td className="border border-gray-300 px-4 py-2">Masse commune</td><td className="border border-gray-300 px-4 py-2">Masse commune</td></tr>
          </tbody>
        </table>
      </div>
      
      <p className="mt-4">
        • I2C en mode multi-esclaves possible <br />
        • Le maître contrôle l'envoi des mesures <br />
        • L'esclave met à jour l'écran LCD à chaque réception <br />
        • Fréquence I2C : 100 kHz (standard)
      </p>

      <h2 id="fichiers-kicad" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Fichiers KiCad
      </h2>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">Cube (Boîte Noire)</h3>
      <p>
        • <code>boite_black.kicad_sch</code> : schéma logique <br />
        • <code>boite_black.kicad_pcb</code> : PCB routé avec zone GND <br />
        • <b>Utilisé comme modèle pour veroboard</b>
      </p>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">Station de Contrôle</h3>
      <p>
        • <code>station_controle.kicad_sch</code> : schéma I2C + LCD <br />
        • <code>station_controle.kicad_pcb</code> : PCB propre <br />
        • <b>Utilisé comme base de placement manuel</b>
      </p>

      <h2 id="tests-simulations" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Tests et Simulations
      </h2>
      <p>
        <span className="text-green-600">✅</span> Test de continuité sur veroboard <br />
        <span className="text-green-600">✅</span> Lecture des valeurs MPU6050 → transmission I2C <br />
        <span className="text-green-600">✅</span> Affichage LCD 16x2 correct (position, vitesse) <br />
        <span className="text-green-600">✅</span> Schémas validés par DRC = 0 erreur <br />
        <span className="text-blue-600">🛠</span> Tests réalisés avec Arduino IDE (hors présentation finale) <br />
        <span className="text-green-600">✅</span> Validation de la stabilité de l'alimentation 5V
      </p>

      <h2 id="methodologie" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Méthodologie du Projet
      </h2>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">1. Analyse du cahier des charges</h3>
      <p>
        Lecture complète des exigences techniques (boîte noire + station) <br />
        Identification des contraintes : pas d'Arduino, pas de breadboard <br />
        Définition de l'architecture : maître I2C (cube) ↔ esclave I2C (station LCD)
      </p>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">2. Conception des schémas (KiCad)</h3>
      <p>
        Création de deux projets séparés : <br />
        • <code>boite_black.kicad_sch</code> pour le cube <br />
        • <code>station_controle.kicad_sch</code> pour la station <br />
        Choix des composants adaptés aux contraintes d'intégration (cube 7×7 cm)
      </p>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">3. Attribution des empreintes</h3>
      <p>
        Sélection des empreintes THT (Through Hole) compatibles veroboard <br />
        Attribution logique dans l'éditeur schématique
      </p>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">4. Routage PCB (perspective future)</h3>
      <p>
        Organisation claire des composants dans l'éditeur PCB <br />
        Ajout d'un plan de masse GND <br />
        Vérification via DRC (Design Rule Check) → 0 erreur
      </p>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">5. Réalisation sur veroboard</h3>
      <p>
        Utilisation du placement PCB comme guide visuel <br />
        Report manuel sur plaque à bandes (veroboard) <br />
        Soudures, coupes de bandes et connexions I2C
      </p>

      <h3 className="text-lg font-semibold text-blue-700 mt-4 mb-2">6. Tests de fonctionnement</h3>
      <p>
        Chargement du code maître / esclave via Arduino IDE <br />
        Lecture des données MPU6050 sur Cube <br />
        Réception et affichage sur LCD via I2C <br />
        Vérification de la stabilité de l'alimentation
      </p>

      <h2 id="notes-equipe-it2" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Notes de l'Équipe
      </h2>
      <p>
        Ce projet nous a permis de comprendre l'interfaçage I2C entre microcontrôleurs et de créer une solution de visualisation temps réel. <br /><br />
        Nous avons acquis une expérience précieuse dans la conception de circuits sur KiCad et leur réalisation manuelle sur veroboard. <br /><br />
        La contrainte de ne pas utiliser Arduino en présentation finale nous a poussés à créer une solution plus professionnelle et compacte. <br /><br />
        Les tests de communication I2C ont été cruciaux pour valider le bon fonctionnement du système maître-esclave.
      </p>

      <h2 id="etapes-completees2" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Étapes Complétées
      </h2>
      <p>
        <span className="text-green-600">✅</span> Conception des schémas KiCad (cube et station) <br />
        <span className="text-green-600">✅</span> Routage des PCB pour référence future <br />
        <span className="text-green-600">✅</span> Réalisation des circuits sur veroboard <br />
        <span className="text-green-600">✅</span> Programmation des microcontrôleurs ATmega328P <br />
        <span className="text-green-600">✅</span> Tests de communication I2C <br />
        <span className="text-green-600">✅</span> Validation de l'affichage LCD <br />
        <span className="text-green-600">✅</span> Documentation technique complète <br />
        <span className="text-blue-600">🔄</span> Intégration dans le cube final (en cours)
      </p>

      <h2 id="fichiers-inclus2" className="text-xl font-semibold text-blue-800 mt-8 mb-3">
        Fichiers Inclus dans le Dépôt GitHub
      </h2>
      <FileLinks />

      <div className="mt-8 p-4 bg-green-50 border border-green-200 rounded-lg">
        <h3 className="text-lg font-semibold text-green-800 mb-2">✅ Conclusion</h3>
        <p className="text-green-700">
          Le montage final est <b>100% fonctionnel sur veroboard</b>. <br />
          Les fichiers KiCad sont prêts pour une fabrication professionnelle. <br />
          Le système de boîte noire répond parfaitement aux exigences du cahier des charges. <br />
          La communication I2C entre le cube et la station de contrôle est stable et fiable.
        </p>
      </div>

      <div className="mt-6 p-4 bg-blue-50 border border-blue-200 rounded-lg">
        <h3 className="text-lg font-semibold text-blue-800 mb-2">🔮 Perspectives</h3>
        <p className="text-blue-700">
          🖨 <b>Fabrication PCB</b> en vue de la version finale <br />
          🧩 Intégration dans un cube transparent 7×7×7 cm <br />
          📦 Présentation propre sans breadboard <br />
          💡 Ajout futur de <b>EEPROM externe</b> pour enregistrement des données <br />
          📊 Interface web pour visualisation en temps réel <br />
          🔋 Optimisation de la consommation énergétique
        </p>
      </div>

      <div className="mt-6 p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
        <h3 className="text-lg font-semibold text-yellow-800 mb-2">📁 Structure du Projet</h3>
        <CodeViewer
        code={`projet-boite-noire/
├── README.md
├── boite_black.kicad_sch
├── boite_black.kicad_pcb
├── station_controle.kicad_sch
├── station_controle.kicad_pcb
├── code/
│   ├── cube_master.ino
│   └── station_slave.ino
├── gerbers/
│   └── gerbers.zip
├── photos/
│   ├── cube_veroboard_photos.zip
│   └── station_photos.zip
└── documentation/
    └── documentation_technique.pdf`}
        language="bash" // 
        className="my-4 border-2 border-blue-900/20"
      />
      </div>
      

    </div>
  );
}