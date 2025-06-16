import { CodeViewer } from "../../components/CodeViewer";
import { useState } from "react";

export const TOC = () => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <div className="bg-gray-50 border border-gray-200 rounded-lg p-4 mb-6">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-semibold text-gray-800 hover:text-blue-600"
      >
        📚 Table des matières
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
            href="#architecture"
            className="block text-blue-600 hover:underline"
          >
            2. Architecture du projet
          </a>
          <a href="#classes" className="block text-blue-600 hover:underline">
            3. Description des classes
          </a>
          <a
            href="#installation"
            className="block text-blue-600 hover:underline"
          >
            4. Installation et utilisation
          </a>
          <a href="#exemples" className="block text-blue-600 hover:underline">
            5. Exemples d'utilisation
          </a>
          <a href="#uml" className="block text-blue-600 hover:underline">
            6. Diagramme UML
          </a>
          <a href="#tests" className="block text-blue-600 hover:underline">
            7. Tests et validation
          </a>
          <a href="#conclusion" className="block text-blue-600 hover:underline">
            8. Conclusion
          </a>
        </div>
      )}
    </div>
  );
};

interface CodeExampleProps {
  title: string;
  code: string;
  description: string;
}

export const CodeExample = ({ title, code, description }: CodeExampleProps) => (
  <div className="bg-white border border-gray-200 rounded-lg my-4">
    <div className="bg-gray-50 px-4 py-2 border-b border-gray-200 font-semibold text-gray-800">
      {title}
    </div>
    <div className="p-4">
      <div className="text-sm text-gray-600 mb-3">{description}</div>
      <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm overflow-x-auto">
        <code>{code}</code>
      </pre>
    </div>
  </div>
);

export const ArchitectureOverview = () => (
  <div className="bg-gradient-to-br from-purple-50 to-blue-50 border border-purple-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-purple-800 mb-4">Structure du système</h3>
    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
      <div className="bg-white p-4 rounded-lg border border-purple-200">
        <div className="font-semibold text-purple-700 mb-2">
          🤖 Classe de base
        </div>
        <div className="text-sm text-gray-600">
          <strong>Robots</strong>
          <br />
          Gère les fonctionnalités communes : énergie, position, déplacements
        </div>
      </div>
      <div className="bg-white p-4 rounded-lg border border-green-200">
        <div className="font-semibold text-green-700 mb-2">
          🏠 Robot Domestique
        </div>
        <div className="text-sm text-gray-600">
          <strong>RobotDomestique</strong>
          <br />
          Spécialisé dans les tâches ménagères
        </div>
      </div>
      <div className="bg-white p-4 rounded-lg border border-orange-200">
        <div className="font-semibold text-orange-700 mb-2">
          🏭 Robot Industriel
        </div>
        <div className="text-sm text-gray-600">
          <strong>RobotIndustriel</strong>
          <br />
          Conçu pour la manutention industrielle
        </div>
      </div>
    </div>
  </div>
);

export const RobotsClassDetails = () => (
  <div className="bg-white border border-gray-200 rounded-lg p-6 my-6">
    <h4 className="font-bold text-gray-800 mb-4">Attributs privés</h4>
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-6">
      <div className="bg-gray-50 p-3 rounded">
        <code className="text-blue-600">__energie</code>
        <div className="text-sm text-gray-600 mt-1">
          Niveau d'énergie (0-200)
        </div>
      </div>
      <div className="bg-gray-50 p-3 rounded">
        <code className="text-blue-600">__x, __y</code>
        <div className="text-sm text-gray-600 mt-1">
          Coordonnées de position
        </div>
      </div>
    </div>

    <h4 className="font-bold text-gray-800 mb-4">Méthodes principales</h4>
    <div className="space-y-3">
      {[
        { name: "move(x, y)", desc: "Déplace le robot (coût : 10 énergie)" },
        { name: "recharger()", desc: "Remet l'énergie à 100%" },
        { name: "show()", desc: "Affiche l'état du robot" },
        { name: "action_speciale()", desc: "Action polymorphe à redéfinir" },
        { name: "travailler()", desc: "Comportement de travail générique" },
      ].map((method, index) => (
        <div
          key={index}
          className="flex items-start space-x-3 p-3 bg-blue-50 rounded"
        >
          <code className="text-blue-700 font-mono text-sm">{method.name}</code>
          <span className="text-gray-600 text-sm">{method.desc}</span>
        </div>
      ))}
    </div>
  </div>
);

export const InstallationSteps = () => (
  <div className="bg-gray-50 border border-gray-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-gray-800 mb-4">🚀 Étapes d'installation</h3>
    <div className="space-y-4">
      <div className="flex items-start space-x-3">
        <div className="bg-blue-500 text-white rounded-full w-6 h-6 flex items-center justify-center text-sm font-bold">
          1
        </div>
        <div>
          <div className="font-medium">Prérequis</div>
          <div className="text-sm text-gray-600">
            Python 3.7+ installé sur votre système
          </div>
        </div>
      </div>
      <div className="flex items-start space-x-3">
        <div className="bg-blue-500 text-white rounded-full w-6 h-6 flex items-center justify-center text-sm font-bold">
          2
        </div>
        <div>
          <div className="font-medium">Téléchargement</div>
          <div className="text-sm text-gray-600">
            Cloner ou télécharger le fichier <code>robots.py</code>
          </div>
        </div>
      </div>
      <div className="flex items-start space-x-3">
        <div className="bg-blue-500 text-white rounded-full w-6 h-6 flex items-center justify-center text-sm font-bold">
          3
        </div>
        <div>
          <div className="font-medium">Exécution</div>
          <div className="text-sm text-gray-600">
            Lancer avec <code>python robots.py</code>
          </div>
        </div>
      </div>
    </div>
  </div>
);

export const UMLDiagram = () => (
  <div className="bg-white border border-gray-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-gray-800 mb-4">
      📊 Architecture des classes
    </h3>
    <div className="bg-gray-50 p-4 rounded-lg font-mono text-sm">
      <div className="text-center mb-4">
        <div className="bg-blue-100 border border-blue-300 p-3 rounded inline-block">
          <div className="font-bold text-blue-800">Robots</div>
          <div className="text-xs text-blue-600 mt-1">Classe de base</div>
          <hr className="my-2 border-blue-300" />
          <div className="text-left text-xs space-y-1">
            <div>- __energie: int</div>
            <div>- __x: int</div>
            <div>- __y: int</div>
            <div>+ nom: str</div>
            <div>+ couleur: str</div>
          </div>
          <hr className="my-2 border-blue-300" />
          <div className="text-left text-xs space-y-1">
            <div>+ move(x, y)</div>
            <div>+ recharger()</div>
            <div>+ show()</div>
            <div>+ action_speciale()</div>
            <div>+ travailler()</div>
          </div>
        </div>
      </div>

      <div className="flex justify-center mb-2">
        <div className="border-l-2 border-gray-400 h-8"></div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
        <div className="text-center">
          <div className="bg-green-100 border border-green-300 p-3 rounded inline-block">
            <div className="font-bold text-green-800">RobotDomestique</div>
            <hr className="my-2 border-green-300" />
            <div className="text-left text-xs space-y-1">
              <div>- __taches_menageres: list</div>
              <div>- __taches_effectuees: list</div>
            </div>
            <hr className="my-2 border-green-300" />
            <div className="text-left text-xs space-y-1">
              <div>+ nettoyer(zone)</div>
              <div>+ ranger(zone)</div>
              <div>+ move(x, y) ⚡</div>
              <div>+ action_speciale() ⚡</div>
              <div>+ travailler() ⚡</div>
            </div>
          </div>
        </div>

        <div className="text-center">
          <div className="bg-orange-100 border border-orange-300 p-3 rounded inline-block">
            <div className="font-bold text-orange-800">RobotIndustriel</div>
            <hr className="my-2 border-orange-300" />
            <div className="text-left text-xs space-y-1">
              <div>- __capacite_charge: int</div>
              <div>- __charge_actuelle: int</div>
            </div>
            <hr className="my-2 border-orange-300" />
            <div className="text-left text-xs space-y-1">
              <div>+ soulever(poids)</div>
              <div>+ deposer()</div>
              <div>+ move(x, y) ⚡</div>
              <div>+ action_speciale() ⚡</div>
              <div>+ travailler() ⚡</div>
            </div>
          </div>
        </div>
      </div>

      <div className="text-center mt-4 text-xs text-gray-600">
        ⚡ = Méthode redéfinie (polymorphisme)
      </div>
    </div>

    <div className="mt-4 text-sm text-gray-600">
      <strong>Relations :</strong>
      <ul className="mt-2 space-y-1">
        <li>
          • <strong>Héritage</strong> : RobotDomestique et RobotIndustriel
          héritent de Robots
        </li>
        <li>
          • <strong>Polymorphisme</strong> : Redéfinition des méthodes move(),
          action_speciale(), travailler()
        </li>
        <li>
          • <strong>Encapsulation</strong> : Attributs privés (__) avec
          propriétés d'accès
        </li>
      </ul>
    </div>
  </div>
);

export const TestScenarios = () => (
  <div className="bg-gradient-to-r from-blue-50 to-purple-50 border border-blue-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-blue-800 mb-4">🧪 Scénarios de test</h3>
    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
      {[
        {
          title: "Test d'encapsulation",
          items: [
            "Validation des propriétés",
            "Contrôle des valeurs limites",
            "Protection des attributs privés",
          ],
          color: "blue",
        },
        {
          title: "Test d'héritage",
          items: [
            "Fonctionnalités héritées",
            "Attributs spécialisés",
            "Constructeurs avec super()",
          ],
          color: "green",
        },
        {
          title: "Test de polymorphisme",
          items: [
            "Redéfinition de move()",
            "Actions spécialisées",
            "Comportements différenciés",
          ],
          color: "purple",
        },
      ].map((test, index) => (
        <div
          key={index}
          className={`bg-white p-4 rounded-lg border border-${test.color}-200`}
        >
          <div className={`font-semibold text-${test.color}-700 mb-3`}>
            {test.title}
          </div>
          <ul className="space-y-1 text-sm text-gray-600">
            {test.items.map((item, i) => (
              <li key={i}>• {item}</li>
            ))}
          </ul>
        </div>
      ))}
    </div>
  </div>
);

export const ProjectSummary = () => (
  <div className="bg-gradient-to-br from-green-50 to-blue-50 border border-green-200 rounded-lg p-6 my-6">
    <h3 className="font-bold text-green-800 mb-4">🎯 Objectifs atteints</h3>

    <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
      <div>
        <h4 className="font-semibold text-green-700 mb-3">
          ✅ Fonctionnalités implémentées
        </h4>
        <ul className="space-y-2 text-sm">
          <li className="flex items-center">
            <span className="w-2 h-2 bg-green-500 rounded-full mr-3"></span>
            Classe principale Robot avec attributs encapsulés
          </li>
          <li className="flex items-center">
            <span className="w-2 h-2 bg-green-500 rounded-full mr-3"></span>
            Deux sous-classes spécialisées
          </li>
          <li className="flex items-center">
            <span className="w-2 h-2 bg-green-500 rounded-full mr-3"></span>
            Méthode move() redéfinie dans chaque sous-classe
          </li>
          <li className="flex items-center">
            <span className="w-2 h-2 bg-green-500 rounded-full mr-3"></span>
            Propriétés Python pour l'encapsulation
          </li>
          <li className="flex items-center">
            <span className="w-2 h-2 bg-green-500 rounded-full mr-3"></span>
            Documentation complète avec UML
          </li>
        </ul>
      </div>

      <div>
        <h4 className="font-semibold text-blue-700 mb-3">
          🔧 Concepts POO démontrés
        </h4>
        <ul className="space-y-2 text-sm">
          <li className="flex items-center">
            <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
            <strong>Encapsulation :</strong> Attributs privés + @property
          </li>
          <li className="flex items-center">
            <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
            <strong>Héritage :</strong> super() et extension de fonctionnalités
          </li>
          <li className="flex items-center">
            <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
            <strong>Polymorphisme :</strong> Redéfinition de méthodes
          </li>
          <li className="flex items-center">
            <span className="w-2 h-2 bg-blue-500 rounded-full mr-3"></span>
            <strong>Abstraction :</strong> Interface commune, implémentations
            spécialisées
          </li>
        </ul>
      </div>
    </div>

    <div className="bg-white p-4 rounded-lg border border-green-200">
      <div className="font-semibold text-green-800 mb-2">
        🚀 Perspectives d'évolution
      </div>
      <div className="text-sm text-green-700 space-y-1">
        <p>• Ajout de nouveaux types de robots (médical, spatial, etc.)</p>
        <p>• Implémentation d'un système de capteurs</p>
        <p>• Interface graphique pour la visualisation</p>
        <p>• Système de communication entre robots</p>
      </div>
    </div>
  </div>
);

// Fichier parent qui utilise ces composants
const Semaine1 = () => {
  return (
    <div className="max-w-4xl mx-auto p-6">
      <h1 className="text-3xl font-bold text-gray-800 mb-6">
        Robot Management System 🤖
      </h1>

      <TOC />

      <section id="introduction" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4">Introduction</h2>
        <div className="bg-blue-50 border-l-4 border-blue-400 p-4 my-4">
          <div className="font-semibold text-blue-800 mb-2">
            🎯 Objectifs du projet :
          </div>
          <ul className="text-blue-700 space-y-1">
            <li>• Créer une architecture flexible de classes Robot</li>
            <li>• Implémenter des sous-classes spécialisées</li>
            <li>
              • Démontrer le polymorphisme avec la méthode <code>move()</code>
            </li>
            <li>• Utiliser l'encapsulation avec des propriétés Python</li>
          </ul>
        </div>
      </section>

      <section id="architecture" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4">
          Architecture du projet
        </h2>
        <ArchitectureOverview />
        <div className="mt-4">
          <p>
            Le système est organisé en <strong>3 classes principales</strong>{" "}
            qui illustrent les concepts de POO :
          </p>
          <ul className="list-disc pl-5 mt-2 space-y-1">
            <li>
              <strong>Classe parent</strong> : <code>Robots</code> -
              Fonctionnalités de base
            </li>
            <li>
              <strong>Classe fille 1</strong> : <code>RobotDomestique</code> -
              Spécialisé pour les tâches ménagères
            </li>
            <li>
              <strong>Classe fille 2</strong> : <code>RobotIndustriel</code> -
              Spécialisé pour la manutention
            </li>
          </ul>
        </div>
      </section>

      <section id="classes" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4">
          Description des classes
        </h2>

        <h3 className="text-xl font-semibold text-gray-800 mt-6 mb-4">
          🤖 Classe <code>Robots</code> (Classe de base)
        </h3>
        <RobotsClassDetails />

        <div className="my-6">
          <p>
            <strong>Encapsulation</strong> : Utilisation de propriétés Python (
            <code>@property</code>) pour contrôler l'accès aux attributs privés.
          </p>
          <CodeViewer
            code={`@property
def energie(self):
    return self.__energie

@energie.setter
def energie(self, valeur):
    if 0 <= valeur <= 200:
        self.__energie = valeur
    else:
        print(f"Erreur: Énergie doit être entre 0 et 200")`}
            language="python"
            className="my-4"
          />
        </div>

        <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
          🏠 Classe <code>RobotDomestique</code>
        </h3>
        <CodeViewer
          code={`class RobotDomestique(Robots):
                        """
                        Robot destiné à exécuter des tâches ménagères.
                        """

                        def __init__(self, nom, couleur, taches_menageres):
                            """
                            Initialise un robot domestique avec une liste de tâches ménagères.
                            """
                            super().__init__(nom, couleur)
                            self.__taches_menageres = taches_menageres
                            self.__taches_effectuees = []

                        @property
                        def taches_menageres(self):
                            """
                            Retourne la liste des tâches ménagères possibles.
                            """
                            return self.__taches_menageres.copy()

                        @property
                        def taches_effectuees(self):
                            """
                            Retourne la liste des tâches déjà effectuées.
                            """
                            return self.__taches_effectuees.copy()

                        def nettoyer(self, zone):
                            """
                            Effectue un nettoyage dans une zone.
                            """
                            self.energie = self.energie - 20
                            self.__taches_effectuees.append(f"nettoyage {zone}")
                            print(f"{self.nom} a nettoyé {zone}")
                                
                        def ranger(self, zone):
                            """
                            Effectue un rangement dans une zone.
                            """
                            self.energie = self.energie - 20
                            self.__taches_effectuees.append(f"rangement {zone}")
                            print(f"{self.nom} a rangé {zone}")

                        def move(self, x, y):
                            """
                            Déplace le robot domestique avec un message contextuel.
                            """
                            print(f"{self.nom} (robot domestique) réfléchit avant de se déplacer...")
                            super().move(x, y)

                        def show(self):
                            """
                            Affiche les informations du robot domestique, y compris les tâches.
                            """
                            super().show()
                            print(f"Tâches ménagères: {', '.join(self.taches_menageres)}")
                            print(f"Tâches effectuées: {len(self.__taches_effectuees)}")

                        def action_speciale(self):
                            """
                            Action spéciale propre aux robots domestiques.
                            """
                            print(f"{self.nom} range et organise l'espace de travail")

                        def travailler(self):
                            """
                            Effectue toutes les tâches ménagères si l'énergie est suffisante.
                            """
                            print(f"{self.nom} commence ses tâches ménagères")
                            for tache in self.__taches_menageres:
                                if self.energie < 20:
                                    print(f"{self.nom} n'a pas assez d'énergie pour continuer les tâches ménagères")
                                    break
                                
                                # Choisie une zone aléatoire pour la tâche
                                zone = random.choice([zone for zone in ["salon", "cuisine", "chambre", "salle de bain"] if tache+" "+zone not in self.__taches_effectuees])
                                
                                if tache == "nettoyage":
                                    self.nettoyer(zone)
                                elif tache == "rangement":
                                    self.ranger(zone)
                                else:
                                    print(f"{self.nom} ne sait pas effectuer la tâche: {tache}")
                            
                            self.action_speciale()`}
          language="python"
          className="my-4"
        />
        <div className="bg-green-50 border border-green-200 rounded-lg p-6 my-6">
          {/* Contenu de RobotDomestique */}

          <div className="text-sm text-gray-700 mt-2">
            <ul className="list-disc pl-5 space-y-1">
              <li>
                <strong>Attributs privés :</strong>{" "}
                <code>__taches_menageres</code> (liste des tâches possibles),{" "}
                <code>__taches_effectuees</code> (historique des tâches
                réalisées)
              </li>
              <li>
                <strong>Méthodes spécialisées :</strong>{" "}
                <code>nettoyer(zone)</code>, <code>ranger(zone)</code>,{" "}
                <code>travailler()</code> (exécute toutes les tâches si
                l'énergie est suffisante)
              </li>
              <li>
                <strong>Polymorphisme :</strong> Redéfinition de{" "}
                <code>move()</code>, <code>show()</code> et{" "}
                <code>action_speciale()</code>
              </li>
              <li>
                <strong>Utilisation de propriétés Python</strong> pour l'accès
                sécurisé aux listes de tâches
              </li>
            </ul>
          </div>
        </div>

        <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
          🏭 Classe <code>RobotIndustriel</code>
        </h3>
        <CodeViewer
          code={`class RobotIndustriel(Robots):
                """
                Robot conçu pour des tâches industrielles de manutention.
                """

                def __init__(self, nom, couleur, capacite_charge):
                        """
                        Initialise un robot industriel avec une capacité de charge.
                        """
                        super().__init__(nom, couleur)
                        self.__capacite_charge = capacite_charge
                        self.__charge_actuelle = 0

                @property
                def capacite_charge(self):
                        """
                        Retourne la capacité maximale de charge du robot.
                        """
                        return self.__capacite_charge

                @capacite_charge.setter
                def capacite_charge(self, valeur):
                        """
                        Modifie la capacité de charge si la valeur est positive.
                        """
                        if valeur > 0:
                                self.__capacite_charge = valeur
                        else:
                                print("Erreur: La capacité de charge doit être positive")

                @property
                def charge_actuelle(self):
                        """
                        Retourne le poids actuellement soulevé par le robot.
                        """
                        return self.__charge_actuelle

                def soulever(self, poids):
                        """
                        Soulève une charge si elle est dans la capacité et que l'énergie est suffisante.
                        """
                        if poids <= self.__capacite_charge and self.energie >= 25:
                                self.__charge_actuelle = poids
                                self.energie = self.energie - 25
                                print(f"{self.nom} a soulevé {poids} kg")
                        elif poids > self.__capacite_charge:
                                print(f"{self.nom} ne peut pas soulever {poids} kg (capacité max: {self.__capacite_charge} kg)")
                        else:
                                print(f"{self.nom} n'a pas assez d'énergie pour soulever")

                def deposer(self):
                        """
                        Dépose la charge actuelle si le robot en porte une.
                        """
                        if self.__charge_actuelle > 0:
                                print(f"{self.nom} a déposé {self.__charge_actuelle} kg")
                                self.__charge_actuelle = 0
                        else:
                                print(f"{self.nom} ne porte rien")

                def move(self, x, y):
                        """
                        Déplace le robot industriel, plus lentement s’il porte une charge.
                        """
                        if self.charge_actuelle > 0:
                                print(f"{self.nom} (robot industriel) se déplace lentement avec une charge.")
                        else:
                                print(f"{self.nom} (robot industriel) se déplace rapidement.")
                        super().move(x, y)

                def show(self):
                        """
                        Affiche les informations du robot industriel, y compris la charge.
                        """
                        super().show()
                        print(f"Capacité de charge: {self.__capacite_charge} kg")
                        print(f"Charge actuelle: {self.__charge_actuelle} kg")

                def action_speciale(self):
                        """
                        Action spéciale propre aux robots industriels.
                        """
                        print(f"{self.nom} calibre ses capteurs de poids et vérifie ses systèmes hydrauliques")

                def travailler(self):
                        """
                        Effectue une tâche de manutention en soulevant et déposant une charge.
                        """
                        print(f"{self.nom} commence ses opérations industrielles")
                        poids = random.randint(10, self.__capacite_charge)
                        self.soulever(poids)
                        if self.__charge_actuelle > 0:
                                self.move(random.randint(1, 10), random.randint(1, 10))
                                self.deposer()
                        self.action_speciale()`}
          language="python"
          className="my-4"
        />
        <div className="bg-orange-50 border border-orange-200 rounded-lg p-6 my-6">
          {/* Contenu de RobotIndustriel */}
          <div className="text-sm text-gray-700 mt-2">
            <ul className="list-disc pl-5 space-y-1">
              <li>
                <strong>Attributs privés :</strong>{" "}
                <code>__capacite_charge</code> (capacité maximale),{" "}
                <code>__charge_actuelle</code> (poids porté)
              </li>
              <li>
                <strong>Méthodes spécialisées :</strong>{" "}
                <code>soulever(poids)</code>, <code>deposer()</code>,{" "}
                <code>travailler()</code> (cycle de manutention)
              </li>
              <li>
                <strong>Polymorphisme :</strong> Redéfinition de{" "}
                <code>move()</code>, <code>show()</code> et{" "}
                <code>action_speciale()</code>
              </li>
              <li>
                <strong>Utilisation de propriétés Python</strong> pour l'accès
                sécurisé à la capacité et la charge
              </li>
            </ul>
          </div>
        </div>
      </section>

      <section id="installation" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4">
          Installation et utilisation
        </h2>
        <InstallationSteps />

        <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
          📋 Dépendances
        </h3>
        <p>Le projet utilise uniquement des modules Python standard :</p>
        <ul className="list-disc pl-5 mt-2">
          <li>
            <code>random</code> : Pour la génération de valeurs aléatoires dans
            les tâches
          </li>
        </ul>
      </section>

    <section id="exemples" className="mb-12">
      <h2 className="text-2xl font-bold text-gray-800 mb-4">
        Exemples d'utilisation
      </h2>

      <h3 className="text-xl font-semibold text-gray-800 mt-6 mb-4">
        🤖 Utilisation basique - Robot générique
      </h3>
      <CodeViewer
        language="python"
        code={`# Création d'un robot de base
robot1 = Robots("R2D2", "bleu")

# Affichage de l'état initial
robot1.show()

# Déplacement
robot1.move(5, 3)
robot1.move(10, 7)

# Vérification de l'énergie
print(f"Énergie restante: {robot1.energie}%")

# Recharge
robot1.recharger()
robot1.show()`}
        className="my-4"
      />

      <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
        🏠 Robot Domestique
      </h3>
      <CodeViewer
        language="python"
        code={`# Création d'un robot domestique
robot_maison = RobotDomestique(
    "HomeBot", 
    "blanc", 
    ["nettoyage", "rangement"]
)

# Affichage des capacités
robot_maison.show()

# Exécution de tâches spécifiques
robot_maison.nettoyer("salon")
robot_maison.ranger("cuisine")

# Exécution automatique de toutes les tâches
robot_maison.travailler()

# Vérification des tâches effectuées
print(f"Tâches réalisées: {len(robot_maison.taches_effectuees)}")`}
        className="my-4"
      />

      <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
        🏭 Robot Industriel
      </h3>
      <CodeViewer
        language="python"
        code={`# Création d'un robot industriel
robot_usine = RobotIndustriel("IndustryBot", "jaune", 100)

# Affichage des spécifications
robot_usine.show()

# Opérations de manutention
robot_usine.soulever(75)
robot_usine.move(8, 12)  # Déplacement avec charge
robot_usine.deposer()

# Cycle de travail automatique
robot_usine.travailler()

# Vérification de la capacité
print(f"Capacité: {robot_usine.capacite_charge} kg")`}
        className="my-4"
      />

      <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
        🔄 Démonstration du polymorphisme
      </h3>
      <CodeViewer
        language="python"
        code={`robots = [
    Robots("BasicBot", "gris"),
    RobotDomestique("CleanBot", "blanc", ["nettoyage"]),
    RobotIndustriel("PowerBot", "rouge", 150)
]

# Même appel de méthode, comportements différents
for robot in robots:
    print(f"\\n--- Déplacement de {robot.nom} ---")
    robot.move(5, 5)  # Polymorphisme en action !`}
        className="my-4"
      />
    </section>

      <section id="uml" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4">Diagramme UML</h2>
        <UMLDiagram />
      </section>

      <section id="tests" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4">
          Tests et validation
        </h2>
        <TestScenarios />

        <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
          ✅ Script de test complet
        </h3>
        <CodeViewer
          language="python"
          code={`def test_robot_system():
    print("=== TEST DU SYSTÈME DE ROBOTS ===\\n")
    
    # Test 1: Robot de base
    print("1. TEST ROBOT DE BASE")
    robot_base = Robots("TestBot", "bleu")
    robot_base.show()
    robot_base.move(3, 4)
    print(f"Position: ({robot_base.x}, {robot_base.y})\\n")
    
    # Test 2: Robot domestique
    print("2. TEST ROBOT DOMESTIQUE")
    robot_dom = RobotDomestique("CleanBot", "blanc", ["nettoyage", "rangement"])
    robot_dom.show()
    robot_dom.travailler()
    print()
    
    # Test 3: Robot industriel
    print("3. TEST ROBOT INDUSTRIEL")
    robot_ind = RobotIndustriel("PowerBot", "jaune", 200)
    robot_ind.show()
    robot_ind.travailler()
    print()
    
    # Test 4: Polymorphisme
    print("4. TEST POLYMORPHISME")
    robots = [robot_base, robot_dom, robot_ind]
    for robot in robots:
        print(f"--- {robot.nom} ---")
        robot.move(10, 10)
        robot.action_speciale()
        print()

if __name__ == "__main__":
    test_robot_system()`}
        
        className="my-4"/>
      </section>

      <section id="conclusion" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4">Conclusion</h2>
        <ProjectSummary />

        <h3 className="text-xl font-semibold text-gray-800 mt-8 mb-4">
          📁 Structure du projet
        </h3>
        <CodeViewer
          language="text"
          code={`robot-management-system/
    ├── robots.py              # Code principal avec les 3 classes
    ├── README.md             # Cette documentation
    ├── tests/
    │   ├── test_basic.py     # Tests unitaires de base
    │   └── test_advanced.py  # Tests avancés
    └── docs/
        ├── uml_diagram.png   # Diagramme UML
        └── specifications.md # Spécifications détaillées`}
          className="my-4"
        />
      </section>
    </div>
  );
};

export default Semaine1;
