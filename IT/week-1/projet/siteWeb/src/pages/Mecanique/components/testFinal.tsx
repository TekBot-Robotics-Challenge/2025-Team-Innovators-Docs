import { useState } from "react";
import { File, ChevronDown, ChevronUp, Settings, Cpu, Package, Wrench, Shield, Eye } from "lucide-react";
import conveyorImage from "../screenshots/Illustration.jpg";


// Table des matières avec navigation
export const TableOfContents = () => {
  const [isOpen, setIsOpen] = useState(false);

  const sections = [
    { id: "etudes-preliminaires", title: "1. 🔍 Études Préliminaires" },
    { id: "choix-materiaux", title: "2. ⚙️ Choix et Justification des Matériaux" },
    { id: "structure-generale", title: "3. 🧱 Structure Générale du Convoyeur" },
    { id: "modelisation-cao", title: "4. 🧰 Modélisation CAO" },
    { id: "composants-mecaniques", title: "5. 🧩 Liste des Composants Mécaniques" },
    { id: "assemblage", title: "6. 🛠️ Assemblage des Composants" },
    { id: "cotation", title: "7. 📏 Cotation & Tolérances" },
    { id: "simulation", title: "8. 🧪 Simulation et Vérification" },
    { id: "securite", title: "9. ✅ Sécurité et Fiabilité" },
    { id: "illustration", title: "10. 📷 Illustration finale du design" }
  ];

  return (
    <div className="bg-gradient-to-br from-blue-50 to-indigo-50 border border-blue-200 rounded-xl p-6 mb-8 shadow-sm">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-bold text-blue-900 hover:text-blue-700 transition-colors"
      >
        <span className="text-lg">📋 Table des matières</span>
        {isOpen ? <ChevronUp size={20} /> : <ChevronDown size={20} />}
      </button>
      {isOpen && (
        <div className="mt-6 grid grid-cols-1 md:grid-cols-2 gap-2">
          {sections.map((section) => (
            <a
              key={section.id}
              href={`#${section.id}`}
              className="block px-3 py-2 text-blue-700 hover:text-blue-900 hover:bg-blue-100 rounded-lg transition-colors text-sm"
            >
              {section.title}
            </a>
          ))}
        </div>
      )}
    </div>
  );
};

// Composant pour afficher les spécifications techniques
const SpecCard = ({ icon: Icon, title, children }: { icon: React.ElementType, title: string, children: React.ReactNode }) => (
  <div className="bg-white border border-gray-200 rounded-lg p-4 shadow-sm hover:shadow-md transition-shadow">
    <div className="flex items-center gap-3 mb-3">
      <h3 className="font-semibold text-gray-800">{title}</h3>
    </div>
    <div className="text-sm text-gray-600">{children}</div>
  </div>
);

// Tableau des contraintes mécaniques
const ConstraintTable = () => (
  <div className="overflow-x-auto">
    <table className="w-full border-collapse border border-gray-300 rounded-lg overflow-hidden">
      <thead className="bg-gradient-to-r from-blue-600 to-blue-700 text-white">
        <tr>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Élément</th>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Spécification</th>
        </tr>
      </thead>
      <tbody>
        {[
          ["Longueur convoyeur", "650 mm"],
          ["Hauteur tapis", "100 mm"],
          ["Poids du déchet", "Environ 20–30 g"],
          ["Taille déchet", "Cube de 30 mm x 30 mm x 30 mm"],
          ["Mode de collecte", "Manuel, après tri"]
        ].map(([element, spec], i) => (
          <tr key={i} className={i % 2 === 0 ? "bg-gray-50" : "bg-white"}>
            <td className="border border-gray-300 px-4 py-3 font-medium">{element}</td>
            <td className="border border-gray-300 px-4 py-3">{spec}</td>
          </tr>
        ))}
      </tbody>
    </table>
  </div>
);

// Tableau des composants mécaniques
const ComponentTable = () => (
  <div className="overflow-x-auto">
    <table className="w-full border-collapse border border-gray-300 rounded-lg overflow-hidden">
      <thead className="bg-gradient-to-r from-green-600 to-green-700 text-white">
        <tr>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Élément</th>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Matériau recommandé</th>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Remarques</th>
        </tr>
      </thead>
      <tbody>
        {[
          ["Châssis", "Plastic 20x20 mm", "Léger et modulaire"],
          ["Tapis roulant", "Caoutchouc ou PVC", "Surface lisse, largeur ~80 mm"],
          ["Poulies d'entraînement", "PLA/ABS (3D print)", "Diamètre 50 mm, matériau plastique léger"],
          ["Arbre moteur", "PLA/ABS", "Fixé par vis sans tête (en plastic)"],
          ["Support moteur", "PLA imprimé", "Fixe le moteur au châssis, léger"],
          ["Moteur DC avec réducteur", "-", "Fixé côté gauche"],
          ["Supports capteurs", "PLA/ABS", "Inclinaison optimisée pour champ de vision"],
          ["Pieds", "PLA/ABS", "Hauteur totale = 100 mm"]
        ].map(([element, material, remarks], i) => (
          <tr key={i} className={i % 2 === 0 ? "bg-gray-50" : "bg-white"}>
            <td className="border border-gray-300 px-4 py-3 font-medium">{element}</td>
            <td className="border border-gray-300 px-4 py-3">{material}</td>
            <td className="border border-gray-300 px-4 py-3 text-sm">{remarks}</td>
          </tr>
        ))}
      </tbody>
    </table>
  </div>
);

// Tableau des cotations
const DimensionTable = () => (
  <div className="overflow-x-auto">
    <table className="w-full border-collapse border border-gray-300 rounded-lg overflow-hidden">
      <thead className="bg-gradient-to-r from-purple-600 to-purple-700 text-white">
        <tr>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Élément</th>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Dimension</th>
          <th className="border border-gray-300 px-4 py-3 text-left font-semibold">Tolérance</th>
        </tr>
      </thead>
      <tbody>
        {[
          ["Longueur tapis", "650 mm", "± 2 mm"],
          ["Largeur tapis", "80–100 mm", "± 1 mm"],
          ["Hauteur châssis", "100 mm", "± 2 mm"],
          ["Écartement entre bacs", "50 mm", "± 5 mm"],
          ["Taille cube de déchet", "30 x 30 x 30 mm", "± 0.5 mm"]
        ].map(([element, dimension, tolerance], i) => (
          <tr key={i} className={i % 2 === 0 ? "bg-gray-50" : "bg-white"}>
            <td className="border border-gray-300 px-4 py-3 font-medium">{element}</td>
            <td className="border border-gray-300 px-4 py-3">{dimension}</td>
            <td className="border border-gray-300 px-4 py-3 text-green-600 font-medium">{tolerance}</td>
          </tr>
        ))}
      </tbody>
    </table>
  </div>
);

// Composant principal
export default function TestFinal() {
  const [activeSection, setActiveSection] = useState("etudes-preliminaires");

  return (
    <div className="max-w-7xl mx-auto p-6 bg-gray-50 min-h-screen">
      {/* Header */}
      <div className="text-center mb-8">
        <h1 className="text-4xl font-bold text-transparent bg-clip-text bg-gradient-to-r from-blue-600 to-purple-600 mb-4">
          📘 Système de Convoyeur de Tri Intelligent
        </h1>
        <p className="text-gray-600 text-lg">Documentation technique complète</p>
      </div>

      <TableOfContents />

      {/* Section 1: Études Préliminaires */}
      <section id="etudes-preliminaires" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Eye className="text-blue-600" />
          1. 🔍 Études Préliminaires
        </h2>

        <div className="mb-8">
          <h3 className="text-xl font-semibold text-blue-800 mb-4">1.1 Analyse des besoins fonctionnels</h3>
          <div className="bg-white rounded-lg p-6 shadow-sm border border-gray-200">
            <ul className="space-y-3">
              <li className="flex items-start gap-3">
                <span className="w-2 h-2 bg-blue-500 rounded-full mt-2 flex-shrink-0"></span>
                <span>Transporter des déchets représentés par des cubes de 30 mm.</span>
              </li>
              <li className="flex items-start gap-3">
                <span className="w-2 h-2 bg-blue-500 rounded-full mt-2 flex-shrink-0"></span>
                <span>Permettre l'arrêt et le redémarrage automatique du tapis selon détection.</span>
              </li>
              <li className="flex items-start gap-3">
                <span className="w-2 h-2 bg-blue-500 rounded-full mt-2 flex-shrink-0"></span>
                <span>Identifier la couleur du déchet grâce à un capteur pour déterminer le type de tri.</span>
              </li>
              <li className="flex items-start gap-3">
                <span className="w-2 h-2 bg-blue-500 rounded-full mt-2 flex-shrink-0"></span>
                <span>Diriger manuellement les déchets vers les bacs selon l'indication du système de tri.</span>
              </li>
              <li className="flex items-start gap-3">
                <span className="w-2 h-2 bg-blue-500 rounded-full mt-2 flex-shrink-0"></span>
                <span>Fournir une structure stable et accessible pour l'intervention humaine.</span>
              </li>
              <li className="flex items-start gap-3">
                <span className="w-2 h-2 bg-blue-500 rounded-full mt-2 flex-shrink-0"></span>
                <span>Optimiser la sécurité, la légèreté et la fiabilité de l'installation.</span>
              </li>
            </ul>
          </div>
        </div>

        <div className="mb-8">
          <h3 className="text-xl font-semibold text-blue-800 mb-4">1.2 Contraintes mécaniques</h3>
          <ConstraintTable />
        </div>
      </section>

      {/* Section 2: Choix et Justification des Matériaux */}
      <section id="choix-materiaux" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Settings className="text-blue-600" />
          2. ⚙️ Choix et Justification des Matériaux et Composants
        </h2>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          <SpecCard icon={Package} title="Structure">
            <strong>Châssis en plastic</strong> : léger, solide, modulable.
          </SpecCard>

          <SpecCard icon={Settings} title="Tapis roulant">
            <strong>Caoutchouc ou PVC</strong> : bonne adhérence, résistance à l'usure, nettoyage facile.
          </SpecCard>

          <SpecCard icon={Wrench} title="Pièces imprimées (plastique PLA/ABS)">
            Toutes les pièces ont été imprimées en <strong>plastique (PLA ou ABS)</strong> car l'imprimante disponible ne permettait pas d'utiliser d'autres matériaux. Ce choix a aussi l'avantage de produire des pièces <strong>très légères</strong>.
          </SpecCard>

          <SpecCard icon={Cpu} title="Moteur DC avec réducteur">
            Fournit un couple suffisant pour faire avancer le tapis avec une faible consommation.
          </SpecCard>
        </div>

        <div className="mt-8">
          <h3 className="text-xl font-semibold text-blue-800 mb-4">2.5 Capteurs</h3>
          <div className="bg-white rounded-lg p-6 shadow-sm border border-gray-200">
            <ul className="space-y-2">
              <li><strong>Laser KY-008 + photorésistance</strong> pour la détection de présence.</li>
              <li><strong>Capteur de couleur TCS3200 ou TCS34725</strong> pour la reconnaissance des types de déchets.</li>
            </ul>
          </div>
        </div>
      </section>

      {/* Section 3: Structure Générale */}
      <section id="structure-generale" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Package className="text-blue-600" />
          3. 🧱 Structure Générale du Convoyeur
        </h2>

        <div className="bg-gradient-to-br from-blue-50 to-indigo-50 rounded-lg p-6 border border-blue-200">
          <h3 className="text-xl font-semibold text-blue-800 mb-4">3.1 Description globale</h3>
          <p className="text-gray-700 mb-4">
            Le convoyeur est composé d'une <strong>structure modulaire</strong> imprimés en plastique, comprenant :
          </p>
          <ul className="space-y-2 text-gray-700">
            <li>• Un tapis roulant en caoutchouc souple</li>
            <li>• Deux rouleaux (poulies) d'entraînement et de retour</li>
            <li>• Un support moteur et axe de rotation</li>
            <li>• Un châssis rigide avec pieds stabilisateurs</li>
            <li>• Un capteur de présence à l'entrée et un capteur de couleur au centre</li>
          </ul>
        </div>
      </section>

      {/* Section 4: Modélisation CAO */}
      <section id="modelisation-cao" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Cpu className="text-blue-600" />
          4. 🧰 Modélisation CAO – Conception Assistée par Ordinateur
        </h2>

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          <div className="bg-white rounded-lg p-6 shadow-sm border border-gray-200">
            <h3 className="text-xl font-semibold text-blue-800 mb-4">4.1 Logiciel utilisé</h3>
            <p className="text-2xl font-bold text-blue-600">SolidWorks 2025</p>
          </div>

          <div className="bg-white rounded-lg p-6 shadow-sm border border-gray-200">
            <h3 className="text-xl font-semibold text-blue-800 mb-4">4.2 Présentation des modèles 3D</h3>
            <ul className="space-y-2 text-sm text-gray-700">
              <li>• <strong>Châssis du convoyeur</strong> : modélisé avec profilés plastic</li>
              <li>• <strong>Tapis roulant</strong> : surface mobile placée entre deux rouleaux</li>
              <li>• <strong>Poulies et rouleaux</strong> : entraînement via moteur DC</li>
              <li>• <strong>Supports de capteurs</strong> : pièces orientées et adaptées</li>
            </ul>
          </div>
        </div>

        <div className="mt-6 bg-white rounded-lg p-6 shadow-sm border border-gray-200">
          <h3 className="text-xl font-semibold text-blue-800 mb-4">4.3 Assemblage</h3>
          <ul className="space-y-2 text-gray-700">
            <li>• Assemblage final avec insertion des composants électroniques (moteur, capteurs)</li>
            <li>• Vérification des dimensions : 650 mm de long, 100 mm de haut</li>
            <li>• Simulation possible du mouvement dans SolidWorks</li>
            <li>• Visualisation en vue éclatée et animation de fonctionnement</li>
          </ul>
        </div>
      </section>

      {/* Section 5: Composants Mécaniques */}
      <section id="composants-mecaniques" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Wrench className="text-blue-600" />
          5. 🧩 Liste des Composants Mécaniques
        </h2>
        <ComponentTable />
      </section>

      {/* Section 6: Assemblage */}
      <section id="assemblage" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Settings className="text-blue-600" />
          6. 🛠️ Assemblage des Composants
        </h2>

        <div className="space-y-6">
          {[
            {
              step: "1",
              title: "Montage du châssis",
              content: "Couper les profilés plastic à la longueur : 2x 650 mm (longueur) + 2x 100 mm (hauteur). Connecter les montants avec des équerres en aluminium."
            },
            {
              step: "2",
              title: "Installation du tapis roulant",
              content: "Fixer les poulies à chaque extrémité. Monter le tapis sur les poulies avec tension manuelle. Ajouter un tendeur si besoin (ressort ou vis réglable)."
            },
            {
              step: "3",
              title: "Fixation du moteur",
              content: "Installer le moteur à l'arrière gauche du châssis. Coupler le moteur à l'arbre de la poulie d'entraînement (avec accouplement ou courroie crantée)."
            },
            {
              step: "4",
              title: "Intégration des capteurs",
              content: "Capteur laser ou photorésistance à l'entrée. Capteur de couleur au-dessus du tapis, en position fixe, avec éclairage LED pour régularité."
            },
            {
              step: "5",
              title: "Installation des bacs",
              content: "Positionner les bacs sous la sortie du tapis. Laisser un espace de 5 cm entre chaque bac pour éviter le croisement des déchets."
            }
          ].map((item) => (
            <div key={item.step} className="bg-white rounded-lg p-6 shadow-sm border border-gray-200">
              <h3 className="text-lg font-semibold text-blue-800 mb-3 flex items-center gap-3">
                <span className="w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center text-sm font-bold">
                  {item.step}
                </span>
                {item.title}
              </h3>
              <p className="text-gray-700 ml-11">{item.content}</p>
            </div>
          ))}
        </div>
      </section>

      {/* Section 7: Cotation */}
      <section id="cotation" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Package className="text-blue-600" />
          7. 📏 Cotation & Tolérances
        </h2>
        <DimensionTable />
      </section>

      {/* Section 8: Simulation */}
      <section id="simulation" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Cpu className="text-blue-600" />
          8. 🧪 Simulation et Vérification
        </h2>

        <div className="bg-white rounded-lg p-6 shadow-sm border border-gray-200">
          <ul className="space-y-3">
            <li>• Test de rotation du tapis sous contrainte de 100 g</li>
            <li>• Vérification de l'alignement du tapis via guides latéraux</li>
            <li>• Simulation de passage d'un déchet sous capteur</li>
            <li>• Vérification des distances pour la chute dans le bac correct</li>
          </ul>
        </div>
      </section>

      {/* Section 9: Sécurité */}
      <section id="securite" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Shield className="text-blue-600" />
          9. ✅ Sécurité et Fiabilité
        </h2>

        <div className="bg-green-50 rounded-lg p-6 border border-green-200">
          <ul className="space-y-3 text-green-800">
            <li>• Arêtes vives ébavurées ou arrondies</li>
            <li>• Structure stable et non basculante</li>
            <li>• Aucun élément saillant dangereux pour les utilisateurs</li>
            <li>• Facilité de démontage pour maintenance</li>
          </ul>
        </div>
      </section>

      {/* Section 10: Illustration */}
      <section id="illustration" className="mb-12">
        <h2 className="text-2xl font-bold text-blue-900 mb-6 flex items-center gap-3">
          <Eye className="text-blue-600" />
          10. 📷 Illustration finale du design
        </h2>

        <div className="bg-gradient-to-br from-gray-100 to-gray-200 rounded-lg p-8 text-center border border-gray-300">
          <div className="mb-4">
            <img
              src={conveyorImage}
              alt="Illustration du système de convoyeur"
              className="mx-auto w-full object-contain"
            />
          </div>
          <p className="text-gray-600">Illustration finale du système de convoyeur de tri intelligent</p>
        </div>


      </section>

      {/* Footer */}
      <footer className="bg-blue-900 text-white rounded-lg p-6 text-center">
        <h3 className="text-lg font-semibold mb-2">🎯 Projet Terminé</h3>
        <p>Système de Convoyeur de Tri Intelligent - Documentation technique complète</p>
        <p className="text-blue-200 text-sm mt-2">Conçu avec SolidWorks 2025</p>
      </footer>
    </div>
  );
}