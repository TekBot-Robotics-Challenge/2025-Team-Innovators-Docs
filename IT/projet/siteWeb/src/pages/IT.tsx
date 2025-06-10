import { useEffect, useState } from 'react';
import { Code, Database, Cloud, Terminal, ChevronRight, FileText } from 'lucide-react';

interface Section {
  id: string;
  title: string;
  content: string;
}

interface ITData {
  title: string;
  description: string;
  sections: Section[];
}

const IT = () => {
  const [data, setData] = useState<ITData | null>(null);
  const [activeSection, setActiveSection] = useState<string>('');

  // Import des données depuis le fichier JSON
  useEffect(() => {
    // Charger les données depuis le fichier JSON
    import('../data/it.json')
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

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, [data]);

  const scrollToSection = (sectionId: string) => {
    const element = document.getElementById(sectionId);
    if (element) {
      element.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  };

  const getSectionIcon = (sectionId: string) => {
    const icons = {
      logiciels: Code,
      algorithmes: Terminal,
      'base-donnees': Database,
      deployment: Cloud
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
        <section className="px-6 py-16 max-w-7xl mx-auto" data-aos="fade-up" data-aos-delay="200">
          <div className="text-center mb-12">
            <div className="inline-flex items-center gap-2 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent mb-4">
              <Code className="w-8 h-8 text-purple-500 motion-preset-seesaw motion-duration-300" />
              <span className="text-lg font-semibold uppercase tracking-wider">IT • Développement • Innovation</span>
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
            <div className="lg:col-span-1">
              <div className="sticky top-6">
                <div className="bg-white/70 backdrop-blur-lg p-6 rounded-3xl shadow-xl border border-white/20">
                  <h2 className="text-2xl font-bold mb-6 bg-gradient-to-r from-purple-700 to-pink-700 bg-clip-text text-transparent flex items-center gap-2">
                    <FileText className="w-6 h-6 text-purple-600" />
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
                              ? 'bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg scale-105' 
                              : 'hover:bg-purple-50 text-gray-700 hover:scale-102'
                          }`}
                        >
                          <IconComponent className={`w-5 h-5 ${isActive ? 'text-white' : 'text-purple-600'} transition-transform group-hover:scale-110`} />
                          <span className="font-medium">{section.title}</span>
                          <ChevronRight className={`w-4 h-4 ml-auto transition-transform ${isActive ? 'translate-x-1' : 'group-hover:translate-x-1'}`} />
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
                        <div className="w-12 h-12 bg-gradient-to-br from-purple-500 to-pink-500 rounded-2xl flex items-center justify-center group-hover:scale-110 transition-transform duration-300">
                          <IconComponent className="w-6 h-6 text-white" />
                        </div>
                        <h2 className="text-3xl font-bold text-purple-700 group-hover:text-purple-600 transition-colors duration-300">
                          {section.title}
                        </h2>
                      </div>
                      
                      <div className="prose prose-lg max-w-none">
                        <p className="text-gray-700 leading-relaxed text-lg">
                          {section.content}
                        </p>
                      </div>
                      
                      {/* Decorative elements */}
                      <div className="mt-8 flex justify-end">
                        <div className="w-16 h-1 bg-gradient-to-r from-purple-400 to-pink-400 rounded-full opacity-60 group-hover:opacity-100 transition-opacity duration-300"></div>
                      </div>
                    </div>
                  );
                })}
              </div>
            </div>
          </div>
        </div>

        {/* Bottom Call to Action */}
        <section className="px-6 py-16 max-w-4xl mx-auto" data-aos="fade-up" data-aos-delay="250">
          <div className="bg-gradient-to-r from-purple-600 to-pink-600 p-12 rounded-3xl text-white shadow-2xl text-center">
            <Code className="w-16 h-16 mx-auto mb-6 animate-bounce" style={{ animationDuration: '2s' }} />
            <h3 className="text-3xl font-bold mb-4">Prêt à coder ?</h3>
            <p className="text-xl mb-8 opacity-90">Explorez notre code source et contribuez au développement du TechBot</p>
            <button className="bg-white text-purple-600 px-8 py-4 rounded-2xl font-bold text-lg hover:bg-gray-100 hover:scale-105 transition-all duration-300 shadow-lg">
              Accéder au GitHub
            </button>
          </div>
        </section>
      </div>
    </div>
  );
};

export default IT;