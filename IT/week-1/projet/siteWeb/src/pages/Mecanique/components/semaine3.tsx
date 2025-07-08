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
            href="#equipe3"
            className="block text-blue-600 hover:underline"
          >
            1. Équipe
          </a>
          <a
            href="#objectif-test3"
            className="block text-blue-600 hover:underline"
          >
            2. Objectif du Test
          </a>
          <a
            href="#conception-composant"
            className="block text-blue-600 hover:underline"
          >
            3. Partie 1 – Conception de Composant Complexe
            <div className="ml-4 space-y-1">
              <a
                href="#parametres-generaux3"
                className="block text-blue-600 hover:underline"
              >
                3.1. Paramètres Généraux
              </a>
              <a
                href="#aluminium3"
                className="block text-blue-600 hover:underline"
              >
                3.2. Alliage d'Aluminium 1060
              </a>
              <a
                href="#etapes-modelisation"
                className="block text-blue-600 hover:underline"
              >
                3.3. Étapes de Modélisation
              </a>
              <a
                href="#captures-solidworks"
                className="block text-blue-600 hover:underline"
              >
                3.4. Captures d'écran SolidWorks
              </a>
            </div>
          </a>
          <a
            href="#notes3"
            className="block text-blue-600 hover:underline"
          >
            4. Notes de l'Équipe
          </a>
          <a
            href="#fichiers3"
            className="block text-blue-600 hover:underline"
          >
            5. Fichiers Inclus dans le Dépôt GitHub
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
      name: "Pièce T3 Configuration A",
      file: "Piece_T3_Config_A.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/P3_A_Offcel.SLDPRT",
    },
    {
      name: "Pièce T3 Configuration B",
      file: "Piece_T3_Config_B.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/P3_B_Offcel.SLDPRT",
    },
    {
      name: "Pièce T3 Configuration C",
      file: "Piece_T3_Config_C.SLDPRT",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/P3_C_Offcel.SLDPRT",
    },
    {
      name: "Rapport Propriétés Masse Config A",
      file: "Mass_Properties_Config_A.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/MassPA_T3.pdf",
    },
    {
      name: "Rapport Propriétés Masse Config B",
      file: "Mass_Properties_Config_B.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/MassPB_T3.pdf",
    },
    {
      name: "Rapport Propriétés Masse Config C",
      file: "Mass_Properties_Config_C.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/MassPC_T3.pdf",
    },
    /*{
      name: "Tableau des Dimensions",
      file: "Dimension_Table.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/Dimension_Table.pdf",
    },
    {
      name: "Valeurs de Configuration",
      file: "Configuration_Values.pdf",
      type: "document",
      url: "https://tekbot-robotics-challenge.github.io/2025-Team-Innovators-Docs/output3/Configuration_Values.pdf",
    }*/
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

// Import des images (vous devrez ajuster les chemins selon votre structure)
// Étapes de modélisation
import Step1 from "./screenshots3/Steps/Piece_T3_Step1.png";
import Step2 from "./screenshots3/Steps/Piece_T3_Step2.png";
import Step2_2 from "./screenshots3/Steps/Piece_T3_Step2_2.png";
import Step3 from "./screenshots3/Steps/Piece_T3_Step3.png";
import Step3_2 from "./screenshots3/Steps/Piece_T3_Step3_2.png";
import Step4 from "./screenshots3/Steps/Piece_T3_Step4.png";
import Step5 from "./screenshots3/Steps/Piece_T3_Step5.png";
import Step5_2 from "./screenshots3/Steps/Piece_T3_Step5_2.png";
import Step6 from "./screenshots3/Steps/Piece_T3_Step6.png";
import Step7 from "./screenshots3/Steps/Piece_T3_Step7.png";
import Step7_2 from "./screenshots3/Steps/Piece_T3_Step7_2.png";
import Step8 from "./screenshots3/Steps/Piece_T3_Step8.png";
import Step9 from "./screenshots3/Steps/Piece_T3_Step9.png";
import Step9_2 from "./screenshots3/Steps/Piece_T3_Step9_2.png";
import Step9_3 from "./screenshots3/Steps/Piece_T3_Step9_3.png";
import Step9_4 from "./screenshots3/Steps/Piece_T3_Step9_4.png";
import Step10 from "./screenshots3/Steps/Piece_T3_Step10.png";
import Step10_1 from "./screenshots3/Steps/Piece_T3_Step10_1.png";
import Step10_2 from "./screenshots3/Steps/Piece_T3_Step10_2.png";
import Step11 from "./screenshots3/Steps/Piece_T3_Step11.png";
import Step12 from "./screenshots3/Steps/Piece_T3_Step12.png";
import Step13 from "./screenshots3/Steps/Piece_T3_Step13.png";
import Step13_2 from "./screenshots3/Steps/Piece_T3_Step13_2.png";

// Configuration A
import P3_A_Offcel from "./screenshots3/Image_A/P3_A_Offcel.jpg";
import P3_A_Vue_Face from "./screenshots3/Image_A/P3_A_Vue_Face.jpg";
import P3_A_Vue_Haut from "./screenshots3/Image_A/P3_A_Vue_Haut.jpg";

// Configuration B
import P3_B_Offcel from "./screenshots3/Image_B/P3_B_Offcel.png";
import P3_B_Vue_Face from "./screenshots3/Image_B/P3_B_Vue_Face.jpg";
import P3_B_Vue_Haut from "./screenshots3/Image_B/P3_B_Vue_Haut.jpg";

// Configuration C
import P3_C_img from "./screenshots3/Image_C/P3_C_img.jpg";
import P3_C_Vue_Face from "./screenshots3/Image_C/P3_C_Vue_Face.jpg";
import P3_C_Vue_Haut from "./screenshots3/Image_C/P3_C_Vue_Haut.jpg";

export default function Semaine3() {
  return (
    <div className="prose max-w-none">
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-blue-900 mb-2">
          📄 Test Mécanique Avancé | TRC 2025
        </h1>
      </div>
      
      <TableOfContents />
      
      <h2
        id="equipe3"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        👤 Équipe
      </h2>
      <p>
        <b>Nom de l'équipe</b> : Innovator <br />
        <b>Participants</b> : Muriella, Nekena, Vanilah <br />
        <b>Domaine</b> : Mécanique <br />
        <b>Semaine</b> : 3 (20 juin 2025 – 26 juin 2025)
      </p>

      <h2
        id="objectif-test3"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        🎯 Objectif du Test
      </h2>
      <p className="box-border border-2 border-dashed p-4 bg-blue-800 text-white border-blue-800">
        Ce test de niveau avancé évalue la capacité à concevoir une pièce mécanique complexe en réponse à un problème mécanique concret. <br /><br />
        <b>Les objectifs clés incluent :</b><br />
        • Modéliser un composant complexe avec des contraintes géométriques précises<br />
        • Calculer la masse de la pièce sous plusieurs configurations<br />
        • Démontrer la créativité, la rigueur technique et les compétences de résolution de problèmes
      </p>

      <h2
        id="conception-composant"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        🛠️ Partie 1 – Conception de Composant Complexe
      </h2>

      <h3
        id="parametres-generaux3"
        className="text-lg font-semibold text-blue-700 mt-6 mb-3"
      >
        ✅ Paramètres Généraux
      </h3>
      <p>
        • <b>Système d'unités</b> : MMGS (millimètre, gramme, seconde) <br />
        • <b>Précision décimale</b> : 2 <br />
        • <b>Matériau</b> : Alliage d'Aluminium 1060 <br />
        • <b>Densité</b> : 2,7 g/cm³ (0,0027 g/mm³)
      </p>

      <h3
        id="aluminium3"
        className="text-lg font-semibold text-blue-700 mt-6 mb-3"
      >
        🔹 Alliage d'Aluminium 1060
      </h3>
      
      <div className="overflow-x-auto">
        <table className="min-w-full border-collapse border border-gray-300 my-4">
          <thead className="bg-blue-100">
            <tr>
              <th className="border border-gray-300 px-4 py-2 text-left">Config</th>
              <th className="border border-gray-300 px-4 py-2 text-left">A (mm)</th>
              <th className="border border-gray-300 px-4 py-2 text-left">B (mm)</th>
              <th className="border border-gray-300 px-4 py-2 text-left">W (mm)</th>
              <th className="border border-gray-300 px-4 py-2 text-left">X (mm)</th>
              <th className="border border-gray-300 px-4 py-2 text-left">Y (mm)</th>
              <th className="border border-gray-300 px-4 py-2 text-left">Z (mm)</th>
              <th className="border border-gray-300 px-4 py-2 text-left">Masse Calculée (g)</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td className="border border-gray-300 px-4 py-2 font-medium">a</td>
              <td className="border border-gray-300 px-4 py-2">193</td>
              <td className="border border-gray-300 px-4 py-2">88</td>
              <td className="border border-gray-300 px-4 py-2">44</td>
              <td className="border border-gray-300 px-4 py-2">48,25</td>
              <td className="border border-gray-300 px-4 py-2">93,5</td>
              <td className="border border-gray-300 px-4 py-2">103</td>
              <td className="border border-gray-300 px-4 py-2 font-bold">1400,56</td>
            </tr>
            <tr className="bg-gray-50">
              <td className="border border-gray-300 px-4 py-2 font-medium">b</td>
              <td className="border border-gray-300 px-4 py-2">205</td>
              <td className="border border-gray-300 px-4 py-2">100</td>
              <td className="border border-gray-300 px-4 py-2">50</td>
              <td className="border border-gray-300 px-4 py-2">51,25</td>
              <td className="border border-gray-300 px-4 py-2">105,5</td>
              <td className="border border-gray-300 px-4 py-2">115</td>
              <td className="border border-gray-300 px-4 py-2 font-bold">1651,41</td>
            </tr>
            <tr>
              <td className="border border-gray-300 px-4 py-2 font-medium">c</td>
              <td className="border border-gray-300 px-4 py-2">210</td>
              <td className="border border-gray-300 px-4 py-2">105</td>
              <td className="border border-gray-300 px-4 py-2">52,5</td>
              <td className="border border-gray-300 px-4 py-2">52,5</td>
              <td className="border border-gray-300 px-4 py-2">110,5</td>
              <td className="border border-gray-300 px-4 py-2">120</td>
              <td className="border border-gray-300 px-4 py-2 font-bold">1760,42</td>
            </tr>
          </tbody>
        </table>
      </div>

      <h3
        id="etapes-modelisation"
        className="text-lg font-semibold text-blue-700 mt-8 mb-3"
      >
        🪛 Étapes de Modélisation Pas à Pas
      </h3>
      
    <div className="grid grid-cols-1 md:grid-cols-2 gap-6 my-8">
      {[
        { step: "Étape 1", img: Step1, alt: "Étape 1" },
        { step: "Étape 2", img: Step2, alt: "Étape 2" },
        { step: "Étape 2 (alt)", img: Step2_2, alt: "Étape 2 alternative" },
        { step: "Étape 3", img: Step3, alt: "Étape 3" },
        { step: "Étape 3 (alt)", img: Step3_2, alt: "Étape 3 alternative" },
        { step: "Étape 4", img: Step4, alt: "Étape 4" },
        { step: "Étape 5", img: Step5, alt: "Étape 5" },
        { step: "Étape 5 (alt)", img: Step5_2, alt: "Étape 5 alternative" },
        { step: "Étape 6", img: Step6, alt: "Étape 6" },
        { step: "Étape 7", img: Step7, alt: "Étape 7" },
        { step: "Étape 7 (alt)", img: Step7_2, alt: "Étape 7 alternative" },
        { step: "Étape 8", img: Step8, alt: "Étape 8" },
        { step: "Étape 9", img: Step9, alt: "Étape 9" },
        { step: "Étape 9 (alt1)", img: Step9_2, alt: "Étape 9 alternative 1" },
        { step: "Étape 9 (alt2)", img: Step9_3, alt: "Étape 9 alternative 2" },
        { step: "Étape 9 (alt3)", img: Step9_4, alt: "Étape 9 alternative 3" },
        { step: "Étape 10", img: Step10, alt: "Étape 10" },
        { step: "Étape 10 (1)", img: Step10_1, alt: "Étape 10 partie 1" },
        { step: "Étape 10 (2)", img: Step10_2, alt: "Étape 10 partie 2" },
        { step: "Étape 11", img: Step11, alt: "Étape 11" },
        { step: "Étape 12", img: Step12, alt: "Étape 12" },
        { step: "Étape 13", img: Step13, alt: "Étape 13" },
        { step: "Étape 13 (alt)", img: Step13_2, alt: "Étape 13 alternative" },
      ].map(({ step, img, alt }, i) => (
        <div
        key={step}
        className="group bg-gradient-to-br from-blue-50 to-white border border-blue-100 rounded-xl shadow-sm hover:shadow-lg transition-shadow duration-200 p-0 flex flex-col overflow-hidden"
        >
        <div className="relative">
          <img
            src={img}
            alt={alt}
            className="w-full h-56 object-cover object-center transition-transform duration-200 group-hover:scale-105"
            loading="lazy"
            style={{ background: "#f3f4f6" }}
          />
          <span className="absolute top-3 left-3 bg-blue-600 text-white text-xs px-3 py-1 rounded-full shadow font-semibold">
            {i + 1 < 10 ? `0${i + 1}` : i + 1}
          </span>
        </div>
        <div className="flex-1 flex items-center px-5 py-4">
          <h4 className="font-semibold text-blue-800 text-lg">{step}</h4>
        </div>
        </div>
      ))}
    </div>

      <h3
        id="captures-solidworks"
        className="text-lg font-semibold text-blue-700 mt-8 mb-3"
      >
        📸 Captures d'écran SolidWorks
      </h3>

    <div className="space-y-10">
      {/* Configuration A */}
      <div>
        <h4 className="text-lg font-bold text-blue-700 mb-6 flex items-center gap-2">
        <span className="inline-block w-2 h-2 rounded-full bg-blue-400"></span>
        Configuration A
        </h4>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        {[ 
          { label: "Vue Isométrique", img: P3_A_Offcel, alt: "Configuration A - Vue isométrique" },
          { label: "Vue de Face", img: P3_A_Vue_Face, alt: "Configuration A - Vue de face" },
          { label: "Vue de Dessus", img: P3_A_Vue_Haut, alt: "Configuration A - Vue de dessus" }
        ].map(({ label, img, alt }) => (
          <div
            key={label}
            className="relative group rounded-2xl overflow-hidden shadow-lg border border-blue-100 bg-gradient-to-br from-blue-50 to-white hover:shadow-2xl transition-all duration-200"
          >
            <img
            src={img}
            alt={alt}
            className="w-full h-56 object-cover object-center group-hover:scale-105 transition-transform duration-200"
            loading="lazy"
            style={{ background: "#f3f4f6" }}
            />
            <div className="absolute top-3 left-3 bg-blue-600/90 text-white text-xs px-3 py-1 rounded-full shadow font-semibold">
            {label}
            </div>
            <div className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-blue-900/60 to-transparent px-4 py-2">
            <span className="text-white font-medium text-sm drop-shadow">{alt}</span>
            </div>
          </div>
        ))}
        </div>
      </div>

      {/* Configuration B */}
      <div>
        <h4 className="text-lg font-bold text-blue-700 mb-6 flex items-center gap-2">
        <span className="inline-block w-2 h-2 rounded-full bg-blue-400"></span>
        Configuration B
        </h4>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        {[ 
          { label: "Vue Isométrique", img: P3_B_Offcel, alt: "Configuration B - Vue isométrique" },
          { label: "Vue de Face", img: P3_B_Vue_Face, alt: "Configuration B - Vue de face" },
          { label: "Vue de Dessus", img: P3_B_Vue_Haut, alt: "Configuration B - Vue de dessus" }
        ].map(({ label, img, alt }) => (
          <div
            key={label}
            className="relative group rounded-2xl overflow-hidden shadow-lg border border-blue-100 bg-gradient-to-br from-blue-50 to-white hover:shadow-2xl transition-all duration-200"
          >
            <img
            src={img}
            alt={alt}
            className="w-full h-56 object-cover object-center group-hover:scale-105 transition-transform duration-200"
            loading="lazy"
            style={{ background: "#f3f4f6" }}
            />
            <div className="absolute top-3 left-3 bg-blue-600/90 text-white text-xs px-3 py-1 rounded-full shadow font-semibold">
            {label}
            </div>
            <div className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-blue-900/60 to-transparent px-4 py-2">
            <span className="text-white font-medium text-sm drop-shadow">{alt}</span>
            </div>
          </div>
        ))}
        </div>
      </div>

      {/* Configuration C */}
      <div>
        <h4 className="text-lg font-bold text-blue-700 mb-6 flex items-center gap-2">
        <span className="inline-block w-2 h-2 rounded-full bg-blue-400"></span>
        Configuration C
        </h4>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        {[ 
          { label: "Vue Isométrique", img: P3_C_img, alt: "Configuration C - Vue isométrique" },
          { label: "Vue de Face", img: P3_C_Vue_Face, alt: "Configuration C - Vue de face" },
          { label: "Vue de Dessus", img: P3_C_Vue_Haut, alt: "Configuration C - Vue de dessus" }
        ].map(({ label, img, alt }) => (
          <div
            key={label}
            className="relative group rounded-2xl overflow-hidden shadow-lg border border-blue-100 bg-gradient-to-br from-blue-50 to-white hover:shadow-2xl transition-all duration-200"
          >
            <img
            src={img}
            alt={alt}
            className="w-full h-56 object-cover object-center group-hover:scale-105 transition-transform duration-200"
            loading="lazy"
            style={{ background: "#f3f4f6" }}
            />
            <div className="absolute top-3 left-3 bg-blue-600/90 text-white text-xs px-3 py-1 rounded-full shadow font-semibold">
            {label}
            </div>
            <div className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-blue-900/60 to-transparent px-4 py-2">
            <span className="text-white font-medium text-sm drop-shadow">{alt}</span>
            </div>
          </div>
        ))}
        </div>
      </div>
    </div>

      <h2
        id="notes3"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        📝 Notes de l'Équipe
      </h2>
      <p>
        • Cette tâche avancée a testé notre réflexion conceptuelle, notre créativité et l'application des contraintes apprises dans un scénario réel.<br />
        • La modélisation de la pièce a impliqué une conception basée sur des paramètres nécessitant des esquisses pilotées par équations.<br />
        • Nous avons optimisé les esquisses et les fonctionnalités pour assurer la faisabilité de simulation et la préparation à l'impression 3D.
      </p>

      <h2
        id="fichiers3"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        📂 Fichiers Inclus dans le Dépôt GitHub
      </h2>
      <p>
        • Fichiers .SLDPRT pour toutes les configurations de pièces<br />
        • Captures d'écran des modèles<br />
        • Rapports de propriétés de masse (PDF)<br />
        • Tableaux de dimensions et valeurs de configuration
      </p>
      
      <FileLinks />
      
    </div>
    

    
  );
}