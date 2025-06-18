import { useState } from "react";

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
            href="#equipe"
            className="block text-blue-600 hover:underline"
          >
            1. Equipe
          </a>
          <a
            href="#objectif-test"
            className="block text-blue-600 hover:underline"
          >
            2. Objectif du Test
          </a>
         
          <a
            href="#modelisation-pieces"
            className="block text-blue-600 hover:underline"
          >
            3.Partie 1 – Modélisation des Pièces
             <div className="ml-4 space-y-1">
            <a
              href="#parametres-communs"
              className="block text-blue-600 hover:underline"
            >
              3.1. Paramètres Communs
            </a>
            <a
              href="#Acier1"
              className="block text-blue-600 hover:underline"
            >
              3.2. – Acier AISI 1020
            </a>
            <a
              href="#Aluminium1"
              className="block text-blue-600 hover:underline"
            >
              3.2. – Alliage d'aluminium 1060
            </a>
            <a
              href="#Acier2"
              className="block text-blue-600 hover:underline"
            >
              3.3. –  Acier AISI 1020
            </a>
            <a
              href="#Aluminium2"
              className="block text-blue-600 hover:underline"
            >
              3.4. – Alliage d'aluminium 1060
            </a>
          </div>
          </a>
          <a
            href="#assemblage-pince-mecanique"
            className="block text-blue-600 hover:underline"
          >
            4.  Partie 2 – Assemblage de la Pince Mécanique
             <div className="ml-4 space-y-1">
            <a
              href="#etapes-completees"
              className="block text-blue-600 hover:underline"
            >
              4.1. Étapes Complétées
            </a>
            <a
              href="#contraintes-specifiques"
              className="block text-blue-600 hover:underline"
            >
              4.2. –  Contraintes Spécifiques Appliquées 
            </a>
            
            
            
            
          </div>
          </a>
          <a
            href="#resultats-analyse-masse"
            className="block text-blue-600 hover:underline"
          >
            5.  Résultats de l'Analyse de Masse
          </a>
          <a
            href="#notes-personnelles"
            className="block text-blue-600 hover:underline"
          >
            6. Notes Personnelles
          </a>
          <a href="#usage-dmp" className="block text-blue-600 hover:underline">
            7. Fichiers Joints au Dépôt GitHub
          </a>
          
        </div>
      )}
    </div>
  );
};

export const ResourceLinks = () => {
  const resources = [
    {
      title: "Pièce1.SLDPRT",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/Piece1.SLDPRT",
    },
    {
      title: "Pièce2.SLDPRT",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/Piece2.SLDPRT",
    },
    {
      title: "Pièce3.SLDPRT",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/piece3.SLDPRT",
    },
    { title: "Pièce4.SLDPRT", url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/piece4.SLDPRT" },
    {
      title: "Assemblage pince.SLDASM",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/assemblage%20pince.SLDASM",
    },
    {
      title: "piece 1.pdf",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/piece%201.pdf",
    },
    {
      title: "piece 2.pdf",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/piece%202.pdf",
    },
    {
      title: "pincePositionMax.pdf",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/pincePositionMax.pdf",
    },
    {
      title: "massePiece3.pdf",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/massePiece3.pdf",
    },
    {
      title: "massePiece4.pdf",
      url: "https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/blob/main/Meca/week-1/output/massePiece4.pdf",
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


import Pièce1 from "./screenshots/piece1.png";
import Pièce1Final from "./screenshots/piece1final.png";
import Pièce2 from "./screenshots/piece2.png";
import Pièce2Final from "./screenshots/piece2final.png";
import Piece3step1 from "./screenshots/piece3step1.png";
import Piece3step2 from "./screenshots/piece3step2.png";
import Piece3step3 from "./screenshots/piece3step3.png";
import Piece3step4 from "./screenshots/piece3Final.png";
import Piece4step1 from "./screenshots/pice4step1.png";
import Piece4step2 from "./screenshots/piece4step2.png";
import Piece4step3 from "./screenshots/piece4step3.png";
import Piece4step4 from "./screenshots/piece4Final.png";
import assemblage from "./screenshots/assemblage1.png";
import pincePositionMinimal from "./screenshots/pincePositionMinimal.png";
import pincePositionMaximal from "./screenshots/pincePositionMaximal.png";


export default function Semaine1() {
    return (
    <div className="prose max-w-none">
          <TableOfContents />
           <h2
        id="equipe"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Equipe
      </h2>
      <p>
        <b>Nom de l'équipe</b> : Innovator <br />
<b>Participants</b> : Muriella, Nekena <br />
<b>Domaine</b> : Mécanique <br />
<b>Semaine</b> : 1 (5 juin 2025 – 12 juin 2025)
      </p>
      <h2
        id="objectif-test"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Objectif du Test
      </h2>
      <p>
        <b>Concevoir</b> et <b>assembler</b> les pièces mécaniques simples fournies dans le défi afin de : <br /> <br />

-Valider la modélisation 3D de pièces à partir de croquis techniques. <br />
-Calculer la masse de chaque pièce avec une tolérance de ±5%.  <br />
-Réaliser l'assemblage complet d'une pince mécanique.  <br />
-Identifier les coordonnées du centre de masse de l'assemblage dans deux positions clés. <br />
      </p>
       <h2
        id="modelisation-pieces"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
      Partie 1-  Modélisation des Pièces
      </h2>
      <h3
        id="parametres-communs"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
      Paramètres Communs
      </h3>
      <p>
        - <b>Système d'unités</b> : MMGS (millimètre, gramme, seconde) <br />
- <b>Décimales</b> : 2 <br />
- <b>Matériaux utilisés</b> : <br />


      </p>
      <h3
        id="Acier1"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Acier AISI 1020
      </h3>
      <p>
        Masse Calculée : 2900,43 g
        <br />
        Capture d'écran SolidWorks: <br /><br /><br />
        <img src={ Pièce1 } alt="pièces1"  /> <br />
        <img src={ Pièce1Final } alt="pièces1"  />

      </p>
       <h3
        id="Aluminium1"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
         Alliage d'aluminium 1060
      </h3>
      <p>
        Masse Calculée : 290,53 g <br />
Capture d'écran SolidWorks <br /><br /><br />
      </p>
      <p>
         <img src={ Pièce2 } alt="pièces1"  /> <br />
        <img src={ Pièce2Final } alt="pièces1"  />
      </p>
      <h3
        id="Acier2"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Acier AISI 1020
      </h3>
      <p>
        Masse Calculée : 1639,86 g <br />
Captures d'écran SolidWorks <br /><br /><br />
      </p>
      <p>
         <img src={ Piece3step1 } alt="pièces1"  /> <br />
        <img src={ Piece3step2 } alt="pièces1"  /> <br />
        <img src={ Piece3step3 } alt="pièces1"  /> <br />
        <img src={ Piece3step4 } alt="pièces1"  /> <br />
      </p>
            <h3
        id="Aluminium2"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
      Alliage d'aluminium 1060
      </h3>
      <p>
        Masse Calculée : 302,48 g  <br />
Capture d'écran SolidWorks <br /><br /><br />
      </p>
      <p>
         <img src={ Piece4step1 } alt="pièces1"  /> <br />
        <img src={ Piece4step2 } alt="pièces1"  /> <br />
        <img src={ Piece4step3 } alt="pièces1"  /> <br />
        <img src={ Piece4step4 } alt="pièces1"  /> <br />
      </p>
      <h2
        id="assemblage-pince-mecanique"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Partie 2 – Assemblage de la Pince Mécanique
      </h2>
            <h3
        id="etapes-completees"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
         Étapes Complétées
      </h3>
      <p>
        Insertion des composants manquants. <br />
Poursuite du processus d'assemblage. <br />
Vérification du mouvement du cylindre et des bras de la pince. <br />
      </p>
       <h3
        id="contraintes-specifiques"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
     Contraintes Spécifiques Appliquées
      </h3>
      <p>
        Cylindre fixé en positions minimale et maximale pour mesurer le centre de gravité. <br />
Symétrie entre les biellettes : appliquée en utilisant un plan médian et une contrainte de symétrie. <br />
      </p>
      <p>
        <h4

        >
Assemblage 
        </h4>
        <img src={ assemblage } alt="pièces1"  /> <br />
      </p>
       <p>
        <h4

        >
Résultat 
        </h4>
        Position maximale: <br />
        <img src={ pincePositionMaximal } alt="pièces1"  /> <br />
         Position minimale: <br />
        <img src={ pincePositionMinimal } alt="pièces1"  /> <br />
      </p>
        <h2
        id="resultats-analyse-masse"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
         Résultats de l'Analyse de Masse
      </h2> 
      <h3
        id="equipe"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Cylindre en position minimale 
      </h3> 
      <p>
        <b>Centre de masse</b> : <br />

X : 701,67 mm <br />
Y : 903,47 mm <br />
Z : 1322,70 mm <br />
      </p>
      <h3
        id="equipe"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Cylindre en position maximale
      </h3> 
      <p>
       <b> Centre de masse</b> : <br />

X : 705,02 mm <br />
Y : 903,37 mm  <br />
Z : 1322,70 mm  <br />
      </p>
      <h2
        id="notes-personnelles"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
      Notes Personnelles
      </h2> 
      <p>
       - L'installation de SolidWorks a causé un léger retard.
-Le test a aidé à renforcer notre maîtrise des bases de SolidWorks (extrusion, assemblage, contraintes). <br />
-Le concept de symétrie mécanique a nécessité l'utilisation d'un plan de référence central. <br />
-Une attention particulière a été portée à la sélection des matériaux pour assurer des calculs de masse réalistes. <br />
      </p>
      <h2
        id="usage-dmp"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
     Fichiers Joints au Dépôt GitHub
      </h2> 
       <ResourceLinks />
     </div>

    );       
}