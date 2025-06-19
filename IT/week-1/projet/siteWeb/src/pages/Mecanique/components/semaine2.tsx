import { useState } from "react";
import { useCallback } from "react";
import '../style.css';
import { File } from "lucide-react";
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
            href="#equipe2"
            className="block text-blue-600 hover:underline"
          >
            1. Equipe
          </a>
          <a
            href="#objectif-test2"
            className="block text-blue-600 hover:underline"
          >
            2. Objectif du Test
          </a>
         
          <a
            href="#modelisation-composants"
            className="block text-blue-600 hover:underline"
          >
            3.Partie 1 – Modélisation des Composants
             <div className="ml-4 space-y-1">
            <a
              href="#parametres-generaux"
              className="block text-blue-600 hover:underline"
            >
              3.1. Paramètres Généraux
              <div className="ml-4 space-y-1">
                <a
              href="#Acier21"
              className="block text-blue-600 hover:underline"
            >
              3.1.1. – Acier AISI 1020
            </a>
            <a
              href="#Acier22"
              className="block text-blue-600 hover:underline"
            >
              3.1.2. – Acier AISI 1020
            </a>
              </div>
            </a>
            
            <a
              href="#Acier23"
              className="block text-blue-600 hover:underline"
            >
              3.2. –  Acier AISI 1020
            </a>
            <a
              href="#Aluminium21"
              className="block text-blue-600 hover:underline"
            >
              3.3. – Alliage d'aluminium 1060
            </a>
          </div>
          </a>
          <a
            href="#notes2"
            className="block text-blue-600 hover:underline"
          >
            4.  Notes de l'Équipe
            
          </a>
          <a
            href="#etapes1"
            className="block text-blue-600 hover:underline"
          >
            5.  Étapes Complétées
          </a>
          <a
            href="#fichiers2"
            className="block text-blue-600 hover:underline"
          >
            6. Fichiers Inclus dans le Dépôt GitHub
          </a>
         
          
        </div>
      )}
    </div>
  );
};



// Interface pour typer les ressources

type FileItem = {
  name: string;
  file: string;
  type: "code" | "schematic" | "document" | "media";
  url: string; // Ajout d'une URL pour le téléchargement/consultation
};

export const FileLinks = () => {
  const files: FileItem[] = [
    {
      name: "massePart1a",
      file: "massePart1a.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/massePart1a.pdf",
    },
    {
      name: "massePart1b",
      file: "massePart1b.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/massePart1b.pdf",
    },
    {
      name: "MassP2_T2",
      file: "MassP2_T2.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/MassP2_T2.pdf",
    },
     {
      name: "MassP3_T2",
      file: "MassP3_T2.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/MassP3_T2.pdf",
    },
    {
      name: "Part1a",
      file: "Part1a.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/Part1a.SLDPRT",
    },
    {
      name: "Part1b",
      file: "Part1a.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/Part1a.SLDPRT",
    },
    {
      name: "Piece2_T2",
      file: "Piece2_T2.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/Piece2_T2.SLDPRT",
    },
    {
      name: "Piece3_T2",
      file: "Piece3_T2.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output2/Piece3_T2.SLDPRT",
    }
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
      document: "bg-lime-100 text-lime-700",
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
            hover:border-yellow-700 active:scale-[0.98]
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
                <File />
              </span>
            </div>
          </div>
        </div>
      ))}
    </div>
  );
};


import Pièce1_T2_face_A from "./screenshots2/Piece1_T2_face_A.png";
import P1_T2 from "./screenshots2/P1_T2.png";
import P2_T2_C0 from "./screenshots2/P2_T2_C0.png";
import P2_T2_C1 from "./screenshots2/P2_T2_C1.png";
import P2_T2_C2 from "./screenshots2/P2_T2_C2.png";
import P2_T2_C3 from "./screenshots2/P2_T2_C3.png";
import P2_T2 from "./screenshots2/P2_T2.png";
 import P3_T2_C0 from "./screenshots2/P3_T2_C0.png";
 import P3_T2 from "./screenshots2/P3_T2.png";



export default function Semaine2() {
    return (
    <div className="prose max-w-none">
          <TableOfContents />
           <h2
        id="equipe2"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Equipe
      </h2>
      <p>
        <b>Nom de l'équipe</b> : Innovator <br />
<b>Participants</b> : Muriella, Nekena, Vanilah <br />
<b>Domaine</b> : Mécanique <br />
<b>Semaine</b> : 2 (13 juin 2025 – 19 juin 2025)
      </p>
      <h2
        id="objectif-test2"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Objectif du Test
      </h2>
      <p className="box-border border-2 border-dashed p-2 bg-blue-800 text-white border-blue-800">
       Cette évaluation se concentre sur la modélisation et l'assemblage de composants mécaniques de niveau intermédiaire. Les objectifs principaux sont : <br />

Créer des modèles 3D précis basés sur les dessins techniques fournis. <br />
Calculer la masse de chaque pièce, en respectant une tolérance de ±1%. <br />
Réaliser l'assemblage d'un sous-système mécanique. <br />
Déterminer les coordonnées du centre de masse pour l'assemblage dans deux configurations différentes. <br />
      </p>
       <h2
        id="modelisation-composants"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
      Partie 1-  Modélisation des Composants
      </h2>
      <h3
        id="parametres-generaux"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
      Paramètres Généraux
      </h3>
      <p>
        - <b>Système d'unités</b> : MMGS (millimètre, gramme, seconde) <br />
- <b>Décimales</b> : 2 <br />
- <b>Matériaux utilisés</b> : <br />

Acier AISI 1020 : densité 0,0079 g/mm³
      </p>
      <h3
        id="Acier21"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Acier AISI 1020
      </h3>
      <p>
        Masse Calculée : 986.37g
        <br />
        Capture d'écran SolidWorks: <br /><br /><br />
        <img src={ Pièce1_T2_face_A } alt="pièces1"  /> <br />
        <img src={ P1_T2 } alt="pièces1"  />

      </p>
       <h3
        id="Acier22"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
         Acier AISI 1020
      </h3>
      <p>
        Masse Calculée : 1089.37g <br />
Capture d'écran SolidWorks <br /><br /><br />
      </p>
      <p>
         <img src={ Pièce1_T2_face_A } alt="pièces1"  /> <br />
        <img src={ P1_T2 } alt="pièces1"  />
      </p>
      <h3
        id="Acier23"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Acier AISI 1020
      </h3>
      <p>
        <b>Masse Calculée</b> : 635.23g  <br />
Captures d'écran SolidWorks <br /><br /><br />
      </p>
      <p>
         <img src={ P2_T2_C0 } alt="pièces1"  /> <br />
        <img src={ P2_T2_C1 } alt="pièces1"  /> <br />
        <img src={ P2_T2_C2 } alt="pièces1"  /> <br />
        <img src={ P2_T2_C3 } alt="pièces1"  /> <br />
        <img src={ P2_T2 } alt="pièces1"  /> <br />
      </p>
            <h3
        id="Aluminium21"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
      Alliage d'aluminium 1060
      </h3>
      <p>
        Masse Calculée : 553.63g  <br />
Capture d'écran SolidWorks <br /><br /><br />
      </p>
      <p>
         
         <img src={ P3_T2_C0 } alt="pièces1"  /> <br />
        <img src={ P3_T2 } alt="pièces1"  /> <br />
        
      </p>
      <h2
        id="notes2"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Notes de l'Équipe
      </h2>
            
      <p>
        Ce test intermédiaire a nécessité des techniques d'assemblage plus avancées et une gestion des contraintes plus complexe. <br />
Nous avons acquis une expérience supplémentaire avec la géométrie de référence et les contraintes avancées dans SolidWorks. <br />
La sélection précise des matériaux et le calcul de la masse sont restés essentiels pour une modélisation fiable. <br />
      </p>
       
        
      
      <h2
        id="fichiers2"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
     Fichiers Joints au Dépôt GitHub
      </h2> 
       <FileLinks />
     </div>

    );       
}