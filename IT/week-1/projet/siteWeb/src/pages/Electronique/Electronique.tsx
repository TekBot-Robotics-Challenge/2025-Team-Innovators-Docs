import { useEffect, useState } from "react";
import {
  Zap,
  Cpu,
  CircuitBoard,
  Settings,
  ChevronRight,
  FileText,
} from "lucide-react";
import Semaine1 from "./semaine1";
// import Semaine1 from './semaine1.mdx'

// Extend the Window interface to include AOS
declare global {
  interface Window {
    AOS?: {
      refresh: () => void;
    };
  }
}

interface Section {
  id: string;
  title: string;
  content: string;
  link: string;
}

interface ElectroniqueData {
  title: string;
  description: string;
  sections: Section[];
}

const Electronique = () => {
  const [data, setData] = useState<ElectroniqueData | null>(null);
  const [activeSection, setActiveSection] = useState<string>("");

  // Import des données depuis le fichier JSON
  useEffect(() => {
    // Charger les données depuis le fichier JSON
    import("../../data/electronique.json")
      .then((jsonData) => {
        setData(jsonData.default);
      })
      .catch((error) => {
        console.error(
          "Erreur lors du chargement des données électroniques:",
          error
        );
      });
  }, []);

  useEffect(() => {
    // Réinitialiser AOS pour les animations
    if (window.AOS) {
      window.AOS.refresh();
    }

    // Observer pour la navigation active
    const handleScroll = () => {
      if (!data) return;

      const sections = data.sections;
      const scrollPosition = window.scrollY + 100;

      for (const section of sections) {
        const element = document.getElementById(section.id);
        if (element) {
          const offsetTop = element.offsetTop;
          const offsetBottom = offsetTop + element.offsetHeight;

          if (scrollPosition >= offsetTop && scrollPosition < offsetBottom) {
            setActiveSection(section.id);
            break;
          }
        }
      }
    };

    window.addEventListener("scroll", handleScroll);
    return () => window.removeEventListener("scroll", handleScroll);
  }, [data]);

  const scrollToSection = (sectionId: string) => {
    const element = document.getElementById(sectionId);
    if (element) {
      const navbar =
        document.querySelector("nav") ||
        document.querySelector(".navbar") ||
        document.querySelector("header");
      const navbarHeight = navbar ? navbar.offsetHeight : 80; // 80px par défaut si navbar non trouvée

      // Calculer la position avec un offset
      const elementPosition =
        element.getBoundingClientRect().top + window.pageYOffset;
      const offsetPosition = elementPosition - navbarHeight - 20; // -20px pour un peu d'espace supplémentaire

      window.scrollTo({
        top: offsetPosition,
        behavior: "smooth",
      });
    }
  };

  const getSectionIcon = (sectionId: string) => {
    const icons = {
      composants: CircuitBoard,
      circuits: Cpu,
      capteurs: Settings,
      programmation: Zap,
    };
    return icons[sectionId as keyof typeof icons] || FileText;
  };

  if (!data) {
    return (
      <div className="min-h-screen bg-gradient-to-br from-blue-50 via-cyan-50 to-indigo-100 flex items-center justify-center">
        <div className="flex items-center gap-3 text-blue-600">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
          <span className="text-lg font-medium">Chargement...</span>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 via-cyan-50 to-indigo-100 relative overflow-hidden">
      {/* Background decorative elements */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div className="absolute -top-40 -right-40 w-96 h-96 bg-gradient-to-br from-blue-400/20 to-cyan-600/20 rounded-full blur-3xl"></div>
        <div className="absolute -bottom-40 -left-40 w-96 h-96 bg-gradient-to-tr from-indigo-400/20 to-blue-600/20 rounded-full blur-3xl"></div>
        <div className="absolute top-1/3 left-1/4 w-64 h-64 bg-gradient-to-r from-cyan-400/10 to-blue-400/10 rounded-full blur-2xl"></div>
      </div>

      <div className="relative z-10 w-full">
        {/* Hero Section */}
        <section
          className="px-6 py-16 max-w-7xl mx-auto"
          data-aos="fade-up"
          data-aos-delay="200"
        >
          <div className="text-center mb-12">
            <div className="inline-flex items-center gap-2 bg-gradient-to-r from-blue-600 to-cyan-600 bg-clip-text text-transparent mb-4">
              <Zap className="w-8 h-8 text-blue-500 motion-preset-oscillate motion-duration-700" />
              <span className="text-lg font-semibold uppercase tracking-wider">
                Électronique • Circuits • Innovation
              </span>
            </div>

            <h1 className="text-5xl md:text-6xl font-black mb-6 bg-gradient-to-r from-gray-900 via-blue-800 to-cyan-800 bg-clip-text text-transparent leading-tight">
              {data.title}
            </h1>

            <p className="text-xl md:text-2xl text-gray-600 max-w-4xl mx-auto leading-relaxed">
              {data.description}
            </p>
          </div>
        </section>

        {/* Main Content */}
        <div className="px-6 pb-12">
          <div className="max-w-7xl mx-auto grid grid-cols-1 lg:grid-cols-4 gap-8">
            {/* Sidebar Navigation */}
            <div className="lg:col-span-1">
              <div className="sticky">
                <div className="bg-white/70 backdrop-blur-lg p-6 rounded-3xl shadow-xl border border-white/20">
                  <h2 className="text-2xl font-bold mb-6 bg-gradient-to-r from-blue-700 to-cyan-700 bg-clip-text text-transparent flex items-center gap-2">
                    <FileText className="w-6 h-6 text-blue-600" />
                    Sections
                  </h2>

                  <nav className="space-y-2">
                    {data.sections.map((section) => {
                      const IconComponent = getSectionIcon(section.id);
                      const isActive = activeSection === section.id;

                      return (
                        <button
                          key={section.id}
                          onClick={() => scrollToSection(section.id)}
                          className={`w-full text-left p-4 rounded-2xl transition-all duration-300 flex items-center gap-3 group ${
                            isActive
                              ? "bg-gradient-to-r from-blue-500 to-cyan-500 text-white shadow-lg scale-105"
                              : "hover:bg-blue-50 text-gray-700 hover:scale-102"
                          }`}
                        >
                          <IconComponent
                            className={`w-5 h-5 ${
                              isActive ? "text-white" : "text-blue-600"
                            } transition-transform group-hover:scale-110`}
                          />
                          <span className="font-medium">{section.id}</span>
                          <ChevronRight
                            className={`w-4 h-4 ml-auto transition-transform ${
                              isActive
                                ? "translate-x-1"
                                : "group-hover:translate-x-1"
                            }`}
                          />
                        </button>
                      );
                    })}
                  </nav>
                </div>
              </div>
            </div>

            {/* Content Sections */}
            <div className="lg:col-span-3">
              <div className="space-y-8">
                {data.sections.map((section, index) => {
                  const IconComponent = getSectionIcon(section.id);

                  return (
                    <div
                      key={section.id}
                      id={section.id}
                      className="group bg-white/70 backdrop-blur-lg p-8 md:p-12 rounded-3xl shadow-xl border border-white/20 hover:shadow-2xl transition-all duration-500 hover:scale-[1.02]"
                      data-aos="fade-up"
                      data-aos-delay={`${index * 100}`}
                    >
                      <div className="flex items-center gap-4 mb-6">
                        <div className="w-12 h-12 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-2xl flex items-center justify-center group-hover:scale-110 transition-transform duration-300">
                          <IconComponent className="w-6 h-6 text-white" />
                        </div>
                        <h2 className="text-3xl font-bold text-blue-700 group-hover:text-blue-600 transition-colors duration-300">
                          {section.title}
                        </h2>
                      </div>

                      <div className="prose prose-lg max-w-none">
                        {/* <p className="text-gray-700 leading-relaxed text-lg">
                          {section.content}
                        </p>
                        <a href={section.link} className=" text-lg">
                          Voir plus
                        </a> */}
                        <Semaine1 />
                      </div>

                      {/* Decorative elements */}
                      <div className="mt-8 flex justify-end">
                        <div className="w-16 h-1 bg-gradient-to-r from-blue-400 to-cyan-400 rounded-full opacity-60 group-hover:opacity-100 transition-opacity duration-300"></div>
                      </div>
                    </div>
                  );
                })}
              </div>
            </div>
          </div>
        </div>

        {/* Bottom Call to Action */}
        <section
          className="px-6 py-16 max-w-4xl mx-auto"
          data-aos="fade-up"
          data-aos-delay="250"
        >
          <div className="bg-gradient-to-r from-blue-600 to-cyan-600 p-12 rounded-3xl text-white shadow-2xl text-center">
            <Zap className="w-16 h-16 mx-auto mb-6 animate-pulse" />
            <h3 className="text-3xl font-bold mb-4">Prêt à explorer ?</h3>
            <p className="text-xl mb-8 opacity-90">
              Découvrez nos schémas détaillés et commencez votre projet
              électronique
            </p>
            <button className="bg-white text-blue-600 px-8 py-4 rounded-2xl font-bold text-lg hover:bg-gray-100 hover:scale-105 transition-all duration-300 shadow-lg">
              Télécharger les schémas
            </button>
          </div>
        </section>
      </div>
    </div>
  );
};

export default Electronique;
