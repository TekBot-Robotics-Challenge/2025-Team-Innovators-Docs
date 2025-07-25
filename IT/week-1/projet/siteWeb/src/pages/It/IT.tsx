import { useEffect, useState, useRef } from "react";
import {
  Code,
  Database,
  Cloud,
  Terminal,
  ChevronRight,
  FileText,
} from "lucide-react";

import semaineComponents from "./semaineComponents";

interface Section {
  id: string;
  title: string;
}

interface ITData {
  title: string;
  description: string;
  sections: Section[];
}

const IT = () => {
  const [data, setData] = useState<ITData | null>(null);
  const [activeSection, setActiveSection] = useState<string>("");
  const [sidebarStyle, setSidebarStyle] = useState<React.CSSProperties>({
    position: "sticky"
  });
  
  const sidebarRef = useRef<HTMLDivElement>(null);
  const contentRef = useRef<HTMLDivElement>(null);

  // Import des données depuis le fichier JSON
  useEffect(() => {
    // Charger les données depuis le fichier JSON
    import("../../data/it.json")
      .then((jsonData) => {
        setData(jsonData.default);
      })
      .catch((error) => {
        console.error("Erreur lors du chargement des données IT:", error);
      });
  }, []);

  useEffect(() => {
    // Réinitialiser AOS pour les animations
    if (window.AOS) {
      window.AOS.refresh();
    }

    if (!data) return;

    const handleScroll = () => {
      const scrollPosition = window.scrollY;
      const windowHeight = window.innerHeight;
      
      // Calculer la position du sidebar
      const sidebarElement = sidebarRef.current;
      const contentElement = contentRef.current;
      
      if (sidebarElement && contentElement) {
        const sidebarHeight = sidebarElement.offsetHeight;
        const contentRect = contentElement.getBoundingClientRect();
        const contentTop = contentElement.offsetTop;
        const contentBottom = contentTop + contentElement.offsetHeight;
        
        // Position initiale (sticky)
        if (scrollPosition < contentTop - 100) {
          setSidebarStyle({ position: "sticky" });
        }
        // Sidebar suit le scroll
        else if (scrollPosition + sidebarHeight < contentBottom - 100) {
          setSidebarStyle({ 
            position: "fixed", 
            top: "6rem"
          });
        }
        // Sidebar se fixe en bas du contenu
        else {
          setSidebarStyle({ 
            position: "absolute", 
            bottom: "0"
          });
        }
      }

      // Observer pour la navigation active
      const sections = data.sections;
      const scrollPositionWithOffset = window.scrollY + 100;

      for (const section of sections) {
        const element = document.getElementById(section.id);
        if (element) {
          const offsetTop = element.offsetTop;
          const offsetBottom = offsetTop + element.offsetHeight;

          if (scrollPositionWithOffset >= offsetTop && scrollPositionWithOffset < offsetBottom) {
            setActiveSection(section.id);
            break;
          }
        }
      }
    };

    window.addEventListener("scroll", handleScroll);
    handleScroll(); // Appel initial
    return () => window.removeEventListener("scroll", handleScroll);
  }, [data]);

  const scrollToSection = (sectionId: string) => {
    const element = document.getElementById(sectionId);
    if (element) {
      const offsetTop = element.offsetTop - 100; // Offset pour le header
      window.scrollTo({
        top: offsetTop,
        behavior: "smooth"
      });
    }
  };

  const getSectionIcon = (sectionId: string) => {
    const icons = {
      logiciels: Code,
      algorithmes: Terminal,
      "base-donnees": Database,
      deployment: Cloud,
    };
    return icons[sectionId as keyof typeof icons] || FileText;
  };

  if (!data) {
    return (
      <div className="min-h-screen bg-gradient-to-br from-purple-50 via-pink-50 to-indigo-100 flex items-center justify-center">
        <div className="flex items-center gap-3 text-purple-600">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-purple-600"></div>
          <span className="text-lg font-medium">Chargement...</span>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-purple-50 via-pink-50 to-indigo-100 relative overflow-hidden">
      {/* Background decorative elements */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div className="absolute -top-40 -right-40 w-96 h-96 bg-gradient-to-br from-purple-400/20 to-pink-600/20 rounded-full blur-3xl"></div>
        <div className="absolute -bottom-40 -left-40 w-96 h-96 bg-gradient-to-tr from-indigo-400/20 to-purple-600/20 rounded-full blur-3xl"></div>
        <div className="absolute top-1/3 left-1/4 w-64 h-64 bg-gradient-to-r from-pink-400/10 to-purple-400/10 rounded-full blur-2xl"></div>
      </div>

      <div className="relative z-10 w-full">
        {/* Hero Section */}
        <section
          className="px-6 py-16 max-w-7xl mx-auto"
          data-aos="fade-up"
          data-aos-delay="200"
        >
          <div className="text-center mb-12">
            <div className="inline-flex items-center gap-2 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent mb-4">
              <Code className="w-8 h-8 text-purple-500 motion-preset-seesaw motion-duration-300" />
              <span className="text-lg font-semibold uppercase tracking-wider">
                IT • Développement • Innovation
              </span>
            </div>

            <h1 className="text-5xl md:text-6xl font-black mb-6 bg-gradient-to-r from-gray-900 via-purple-800 to-pink-800 bg-clip-text text-transparent leading-tight">
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
            <div className="lg:col-span-1 relative">
              <div 
                ref={sidebarRef}
                style={{
                  position: sidebarStyle.position as 'static' | 'relative' | 'absolute' | 'sticky' | 'fixed',
                  top: sidebarStyle.top,
                  bottom: sidebarStyle.bottom
                }}
                className="w-full lg:w-70 transition-all duration-300 ease-out z-20"
              >
                <div className="bg-white/80 backdrop-blur-xl p-6 rounded-3xl shadow-2xl border border-white/30 hover:bg-white/90 transition-all duration-300">
                  <h2 className="text-2xl font-bold mb-6 bg-gradient-to-r from-purple-700 to-pink-700 bg-clip-text text-transparent flex items-center gap-2">
                    <FileText className="w-6 h-6 text-purple-600" />
                    Sections
                  </h2>

                  <nav className="space-y-3">
                    {data.sections.map((section, index) => {
                      const IconComponent = getSectionIcon(section.id);
                      const isActive = activeSection === section.id;

                      return (
                        <button
                          key={section.id}
                          onClick={() => scrollToSection(section.id)}
                          className={`w-full text-left p-4 rounded-2xl transition-all duration-500 ease-out flex items-center gap-3 group transform hover:scale-[1.02] ${
                            isActive
                              ? "bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg scale-[1.02] animate-pulse"
                              : "hover:bg-gradient-to-r hover:from-purple-50 hover:to-pink-50 text-gray-700 hover:shadow-md"
                          }`}
                          style={{
                            animationDelay: `${index * 100}ms`,
                            transform: isActive ? 'translateX(4px)' : 'translateX(0)'
                          }}
                        >
                          <div className={`p-2 rounded-xl transition-all duration-300 ${
                            isActive 
                              ? "bg-white/20 shadow-inner" 
                              : "bg-purple-100 group-hover:bg-purple-200"
                          }`}>
                            <IconComponent
                              className={`w-4 h-4 ${
                                isActive ? "text-white" : "text-purple-600"
                              } transition-all duration-300 group-hover:scale-110`}
                            />
                          </div>
                          <span className={`font-medium ${
                            isActive ? "text-white" : "text-gray-700"
                          }`}>
                            {section.id}
                          </span>
                          <ChevronRight
                            className={`w-4 h-4 ml-auto transition-all duration-300 ${
                              isActive
                                ? "translate-x-1 text-white rotate-90"
                                : "group-hover:translate-x-1 text-purple-400"
                            }`}
                          />
                        </button>
                      );
                    })}
                  </nav>

                  {/* Progress indicator */}
                  <div className="mt-6 pt-4 border-t border-purple-100">
                    <div className="flex items-center gap-2 text-xs text-gray-500">
                      <div className="flex-1 bg-gray-200 rounded-full h-1">
                        <div 
                          className="bg-gradient-to-r from-purple-500 to-pink-500 h-1 rounded-full transition-all duration-300"
                          style={{ 
                            width: `${((data.sections.findIndex(s => s.id === activeSection) + 1) / data.sections.length) * 100}%`
                          }}
                        ></div>
                      </div>
                      <span className="min-w-max">
                        {data.sections.findIndex(s => s.id === activeSection) + 1}/{data.sections.length}
                      </span>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            {/* Content Sections */}
            <div ref={contentRef} className="lg:col-span-3">
              <div className="space-y-8">
                {data.sections.map((section, index) => {
                  const IconComponent = getSectionIcon(section.id);
                  const SemaineComponent = semaineComponents[index]; // Utilisez la solution 2
                  const isActive = activeSection === section.id;

                  return (
                    <div
                      key={section.id}
                      id={section.id}
                      className={`group bg-white/70 backdrop-blur-lg p-8 md:p-12 rounded-3xl shadow-xl border border-white/20 hover:shadow-2xl transition-all duration-500 hover:scale-[1.02] scroll-mt-24 ${
                        isActive ? "ring-2 ring-purple-300 shadow-2xl scale-[1.01]" : ""
                      }`}
                      
                    >
                      <div className="flex items-center gap-4 mb-6">
                        <div className={`w-12 h-12 bg-gradient-to-br from-purple-500 to-pink-500 rounded-2xl flex items-center justify-center group-hover:scale-110 transition-transform duration-300 ${
                          isActive ? "animate-pulse shadow-lg" : ""
                        }`}>
                          <IconComponent className="w-6 h-6 text-white" />
                        </div>
                        <h2 className={`text-3xl font-bold text-purple-700 group-hover:text-purple-600 transition-colors duration-300 ${
                          isActive ? "text-purple-600" : ""
                        }`}>
                          {section.title}
                        </h2>
                      </div>

                      <div className="prose prose-lg max-w-none">
                        {SemaineComponent && <SemaineComponent />}
                      </div>

                      {/* Decorative elements */}
                      <div className="mt-8 flex justify-end">
                        <div className={`w-16 h-1 bg-gradient-to-r from-purple-400 to-pink-400 rounded-full opacity-60 group-hover:opacity-100 transition-all duration-300 ${
                          isActive ? "w-32 opacity-100" : ""
                        }`}></div>
                      </div>

                      {/* Section number */}
                      <div className="absolute top-6 right-6 w-8 h-8 bg-gradient-to-br from-purple-100 to-pink-100 rounded-full flex items-center justify-center">
                        <span className="text-sm font-bold text-purple-600">
                          {index + 1}
                        </span>
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
          <div className="bg-gradient-to-r from-purple-600 to-pink-600 p-12 rounded-3xl text-white shadow-2xl text-center">
            <Code
              className="w-16 h-16 mx-auto mb-6 animate-bounce"
              style={{ animationDuration: "2s" }}
            />
            <h3 className="text-3xl font-bold mb-4">Prêt à coder ?</h3>
            <p className="text-xl mb-8 opacity-90">
              Explorez notre code source et contribuez au développement du
              TechBot
            </p>
            <a
              href="https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs/tree/60959036d06356c1894f87237db5ecdf9c0bf9b1/IT"
              target="_blank"
              rel="noopener noreferrer"
              className="bg-white text-purple-600 px-8 py-4 rounded-2xl font-bold text-lg hover:bg-gray-100 hover:scale-105 transition-all duration-300 shadow-lg inline-block"
            >
              Accéder au GitHub
            </a>
          </div>
        </section>
      </div>
    </div>
  );
};

export default IT;