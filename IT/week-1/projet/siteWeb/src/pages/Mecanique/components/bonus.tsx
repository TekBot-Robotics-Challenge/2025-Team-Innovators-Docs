import { useState } from "react";
import { useCallback } from "react";
import { File } from "lucide-react";

// Import des screenshots
import challenge2_1 from "./screenshots-bonus/challenge2_1.png";
import challenge2_2 from "./screenshots-bonus/challenge2_2.png";
import challenge2_3 from "./screenshots-bonus/challenge2_3.png";
import challenge2_4 from "./screenshots-bonus/challenge2_4.png";
import challenge2_5 from "./screenshots-bonus/challenge2_5.png";
import challenge2_8 from "./screenshots-bonus/challenge2_8.png";
import challenge2_7 from "./screenshots-bonus/challenge2_7.png";
import challenge2_cut from "./screenshots-bonus/challenge2_cut.png";
import challenge2Fin from "./screenshots-bonus/challenge2Fin.png";

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
            href="#equipe-bonus"
            className="block text-blue-600 hover:underline"
          >
            1. Informations Générales
          </a>
          <a
            href="#objectif-bonus"
            className="block text-blue-600 hover:underline"
          >
            2. Objectif du Challenge
          </a>
          <a
            href="#processus-conception"
            className="block text-blue-600 hover:underline"
          >
            3. Processus de Conception
            <div className="ml-4 space-y-1">
              <a
                href="#analyse-dessin"
                className="block text-blue-600 hover:underline"
              >
                3.1. Analyse du Dessin Technique
              </a>
              <a
                href="#creation-base"
                className="block text-blue-600 hover:underline"
              >
                3.2. Création de la Base
              </a>
              <a
                href="#trous-conges"
                className="block text-blue-600 hover:underline"
              >
                3.3. Trous et Congés
              </a>
              <a
                href="#corps-central"
                className="block text-blue-600 hover:underline"
              >
                3.4. Corps Central
              </a>
              <a
                href="#finitions"
                className="block text-blue-600 hover:underline"
              >
                3.5. Finitions
              </a>
            </div>
          </a>
          <a
            href="#masse-resultat"
            className="block text-blue-600 hover:underline"
          >
            4. Résultat Masse
          </a>
          <a
            href="#illustrations"
            className="block text-blue-600 hover:underline"
          >
            5. Illustrations
          </a>
          <a
            href="#feedback"
            className="block text-blue-600 hover:underline"
          >
            6. Retour d'Expérience
          </a>
          <a
            href="#fichiers-bonus"
            className="block text-blue-600 hover:underline"
          >
            7. Fichiers Inclus
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
};

export const FileLinks = () => {
  const files: FileItem[] = [
    {
      name: "Pièce Challenge 2",
      file: "challenge2.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output-bonus/challenge2.SLDPRT",
    },
    {
      name: "Documentation Technique",
      file: "documentation.md",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output-bonus/documentation.md",
    },
    {
      name: "Rapport Propriétés Masse",
      file: "Mass_Properties_Challenge2.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output-bonus/Mass_Properties_Challenge2.pdf",
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

// Images du processus de modélisation avec imports
const screenshots = [
  { 
    step: "Étape 1", 
    img: challenge2_1, 
    alt: "Esquisse de base",
    description: "Esquisse de base"
  },
  { 
    step: "Étape 2", 
    img: challenge2_2, 
    alt: "Extrusion",
    description: "Extrusion"
  },
  { 
    step: "Étape 3", 
    img: challenge2_3, 
    alt: "Enlèvement de matière et positionnement",
    description: "Enlèvement de matière et positionnement"
  },
  { 
    step: "Étape 4", 
    img: challenge2_4, 
    alt: "Corps central révolutionné",
    description: "Corps central révolutionné"
  },
  { 
    step: "Étape 5", 
    img: challenge2_5, 
    alt: "Détails finaux",
    description: "Détails finaux"
  },
  { 
    step: "Chanfrein", 
    img: challenge2_8, 
    alt: "Exemple de chanfrein",
    description: "Exemple de chanfrein"
  },
  { 
    step: "Vue Isométrique", 
    img: challenge2_7, 
    alt: "Vue isométrique",
    description: "Vue isométrique"
  },
  { 
    step: "Vue en Coupe", 
    img: challenge2_cut, 
    alt: "Vue en coupe",
    description: "Vue en coupe"
  },
  { 
    step: "Rendu Final", 
    img: challenge2Fin, 
    alt: "Rendu final",
    description: "Rendu final"
  }
];

export default function Challenge2() {
  return (
    <div className="prose max-w-none">
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-blue-900 mb-2">
          Challenge II – Conception de Pièce Mécanique (SolidWorks)
        </h1>
      </div>
      
      <TableOfContents />
      
      <h2
        id="equipe-bonus"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Informations Générales
      </h2>
      <div className="bg-blue-50 border border-blue-200 rounded-lg p-4 mb-6">
        <p>
          <b>Nom de l'équipe</b> : Innovator <br />
          <b>Participants</b> : Muriella, Nekena, Vanilah <br />
          <b>Challenge</b> : TRC HEROES – Modélisation de pièce mécanique en SolidWorks <br />
          <b>Date</b> : 24 juin 2025 <br />
          <b>Contraintes</b> :<br />
          • Tous les trous sont traversants sauf indication contraire<br />
          • Unités : MMGS (millimètre, gramme, seconde)<br />
          • Précision décimale : 2<br />
          • Matériau : Alliage d'Aluminium 1060<br />
          • Densité : 0,0079 g/mm³
        </p>
      </div>

      <h2
        id="objectif-bonus"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Objectif du Challenge
      </h2>
      <p className="box-border border-2 border-dashed p-4 bg-blue-800 text-white border-blue-800">
        <b>Objectif</b> : Reproduire fidèlement la pièce mécanique 2D/3D basée sur le dessin technique fourni.<br /><br />
        <b>Compétences évaluées :</b><br />
        • Lecture et interprétation de dessins techniques<br />
        • Maîtrise des outils de modélisation SolidWorks<br />
        • Précision dimensionnelle et géométrique<br />
        • Application des propriétés matériaux
      </p>

      <h2
        id="processus-conception"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Processus de Conception
      </h2>

      <h3
        id="analyse-dessin"
        className="text-lg font-semibold text-blue-700 mt-6 mb-3"
      >
        Étape 1 : Analyse du Dessin Technique
      </h3>
      <p>
        • Examen des différentes vues : Face, Droite, Dessus et Isométrique<br />
        • Prise en compte de toutes les dimensions, vues en coupe et lignes cachées<br />
        • Identification des éléments critiques et des contraintes géométriques
      </p>

      <h3
        id="creation-base"
        className="text-lg font-semibold text-blue-700 mt-6 mb-3"
      >
        Étape 2 : Création de la Base
      </h3>
      <p>
        • Esquisse du profil inférieur sur le plan supérieur<br />
        • Utilisation de l'outil <i>Bossage-Extrusion</i> avec l'épaisseur spécifiée<br />
        • Vérification des dimensions de base
      </p>

      <h3
        id="trous-conges"
        className="text-lg font-semibold text-blue-700 mt-6 mb-3"
      >
        Étape 3 : Trous et Congés
      </h3>
      <p>
        • Création des trous traversants avec positionnement approprié<br />
        • Ajout des congés intérieurs et extérieurs à l'aide de l'outil <i>Congé</i><br />
        • Utilisation de la cotation intelligente pour un placement précis
      </p>

      <h3
        id="corps-central"
        className="text-lg font-semibold text-blue-700 mt-6 mb-3"
      >
        Étape 4 : Modélisation du Corps Central
      </h3>
      <p>
        • Construction de la structure supérieure en utilisant <i>Révolution de Bossage/Base</i><br />
        • Ajout du trou central et de la cavité circulaire supérieure<br />
        • Respect des contraintes de symétrie axiale
      </p>

      <h3
        id="finitions"
        className="text-lg font-semibold text-blue-700 mt-6 mb-3"
      >
        Étape 5 : Finitions
      </h3>
      <p>
        • Vérification de toutes les dimensions globales et arêtes lisses<br />
        • Application du matériau <b>Alliage d'Aluminium 1060</b><br />
        • Utilisation des <i>Propriétés de Masse</i> pour calculer la masse finale
      </p>

      <h2
        id="masse-resultat"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Résultat Masse
      </h2>
      <div className="bg-green-50 border border-green-200 rounded-lg p-4 mb-6">
        <p className="text-center">
            <b>Masse calculée dans SolidWorks</b> :<br />
          <span className="text-2xl font-bold text-green-700">7 903,45 grammes</span>
        </p>
      </div>

      <h2
        id="illustrations"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Illustrations
      </h2>
      <p className="mb-6 text-gray-600">
        Ci-dessous les captures d'écran prises pendant le processus de modélisation :
      </p>
      
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 my-8">
        {screenshots.map(({ step, img, alt, description }, i) => (
          <div
            key={step}
            className="group bg-gradient-to-br from-blue-50 to-white border border-blue-100 rounded-xl shadow-sm hover:shadow-lg transition-shadow duration-200 p-0 flex flex-col overflow-hidden"
          >
            <div className="relative">
              <img
                src={img}
                alt={alt}
                className="w-full h-48 object-cover object-center transition-transform duration-200 group-hover:scale-105"
                loading="lazy"
                style={{ background: "#f3f4f6" }}
              />
              <span className="absolute top-3 left-3 bg-blue-600 text-white text-xs px-3 py-1 rounded-full shadow font-semibold">
                {i + 1 < 10 ? `0${i + 1}` : i + 1}
              </span>
            </div>
            <div className="flex-1 flex flex-col justify-between px-4 py-3">
              <h4 className="font-semibold text-blue-800 text-sm">{step}</h4>
              <p className="text-xs text-gray-600 mt-1">{description}</p>
            </div>
          </div>
        ))}
      </div>

      <h2
        id="feedback"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Retour d'Expérience
      </h2>
      <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-4 mb-6">
        <p>
          • Ce challenge a été une excellente opportunité de renforcer nos compétences en :<br />
          &nbsp;&nbsp;- Opérations de <i>Révolution</i><br />
          &nbsp;&nbsp;- Cotation intelligente<br />
          &nbsp;&nbsp;- Lecture et interprétation de dessins mécaniques<br />
          • Nous avons apprécié l'attention aux détails et la symétrie que cette pièce exigeait.<br />
          • Le défi était accessible avec des compétences SolidWorks de base mais nécessitait une observation pointue et de la discipline.
        </p>
      </div>

      <h2
        id="fichiers-bonus"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        Fichiers Inclus
      </h2>
      <p>
        • <code>challenge2.SLDPRT</code> – Fichier de pièce final<br />
        • <code>documentation.md</code> – Cette documentation<br />
        • <code>screenshot-bonus/</code> – Dossier contenant toutes les images associées
      </p>
      
      <FileLinks />
      
      <div className="bg-blue-50 border border-blue-200 rounded-lg p-4 mt-8">
        <h3 className="text-lg font-semibold text-blue-700 mb-2">Remerciements Spéciaux</h3>
        <p>
          Merci à l'équipe <b>TRC HEROES</b> pour ce challenge engageant et enrichissant !<br />
          <i>Documenté par Mumu.</i>
        </p>
      </div>
    </div>
  );
}