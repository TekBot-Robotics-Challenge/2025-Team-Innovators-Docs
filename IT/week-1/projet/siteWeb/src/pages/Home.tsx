import { useEffect, useState } from 'react';
import { ChevronRight, Zap, Cog, Code, ArrowDown, Sparkles } from 'lucide-react';
import { Link } from 'react-router-dom';

const Home = () => {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(true);
    // Réinitialiser AOS pour s'assurer que les animations fonctionnent après le chargement de la page
    if (window.AOS) {
      window.AOS.refresh();
    }
  }, []);

  const cardData = [
    {
      title: "Électronique",
      description: "Découvrez les circuits, capteurs et composants électroniques utilisés dans le projet TechBot.",
      icon: Zap,
      color: "from-blue-100 to-cyan-100",
      textColor: "text-blue-600",
      href: "/electronique",
      delay: "100"
    },
    {
      title: "Mécanique",
      description: "Explorez la conception mécanique, les matériaux et les techniques d'assemblage du TechBot.",
      icon: Cog,
      color: "from-emerald-100 to-teal-100",
      textColor: "text-emerald-600",
      href: "/mecanique",
      delay: "400"
    },
    {
      title: "IT",
      description: "Apprenez-en plus sur les logiciels, algorithmes et systèmes de contrôle qui animent le TechBot.",
      icon: Code,
      color: "from-purple-100 to-pink-100",
      textColor: "text-purple-600",
      href: "/it",
      delay: "100"
    }
  ];

  return (
    <div className="min-h-screen w-full bg-gradient-to-br from-slate-50 via-blue-50 to-indigo-100 relative overflow-hidden pt-20">
      {/* Background decorative elements */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div className="absolute -top-40 -right-40 w-96 h-96 bg-gradient-to-br from-blue-400/20 to-purple-600/20 rounded-full blur-3xl"></div>
        <div className="absolute -bottom-40 -left-40 w-96 h-96 bg-gradient-to-tr from-emerald-400/20 to-cyan-600/20 rounded-full blur-3xl"></div>
        <div className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-64 h-64 bg-gradient-to-r from-pink-400/10 to-yellow-400/10 rounded-full blur-2xl"></div>
      </div>

      <div className="relative z-10 w-full px-6 py-12">
        {/* Hero Section */}
        <section className="text-center mb-20 max-w-6xl mx-auto">
          <div className={`transform transition-all duration-1000 ${isVisible ? 'translate-y-0 opacity-100' : 'translate-y-10 opacity-0'}`}>
            <div className="inline-flex items-center gap-2 bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent mb-4">
              <Sparkles className="w-6 h-6 text-blue-300 motion-preset-blink motion-duration-1000" />
              <span className="text-sm font-semibold uppercase tracking-wider">Innovation • Technologie • Robotique</span>
            </div>
            
            <h1 className="text-6xl md:text-7xl font-black mb-6 bg-gradient-to-r from-gray-900 via-blue-800 to-purple-800 bg-clip-text text-transparent leading-tight">
              Innovator-TRC
              <span className="block text-4xl md:text-5xl font-light mt-2">Documentation</span>
            </h1>
            
            <p className="text-xl md:text-2xl text-gray-600 max-w-4xl mx-auto leading-relaxed mb-8">
              Bienvenue dans la documentation complète du projet TechBot. Explorez les différentes sections pour découvrir les aspects électroniques, mécaniques et informatiques qui donnent vie à l'innovation.
            </p>
            
            <div className="animate-bounce mt-12">
              <ArrowDown className="w-8 h-8 text-gray-400 mx-auto" />
            </div>
          </div>
        </section>

        {/* About Section */}
        <section className="mb-20 max-w-6xl mx-auto" data-aos="fade-up" data-aos-delay="200">
          <div className="max-w-4xl mx-auto">
            <h2 className="text-4xl md:text-5xl font-bold mb-8 text-center bg-gradient-to-r from-gray-800 to-gray-600 bg-clip-text text-transparent">
              À propos du projet
            </h2>
            
            <div className="bg-white/70 backdrop-blur-lg p-8 md:p-12 rounded-3xl shadow-2xl border border-white/20 hover:shadow-3xl transition-all duration-100 hover:scale-[1.02]">
              <div className="space-y-6 text-lg text-gray-700 leading-relaxed">
                <p className="text-xl font-medium text-gray-800">
                  Le projet TechBot est une initiative multidisciplinaire révolutionnaire qui fusionne harmonieusement électronique, mécanique et informatique pour créer une solution robotique d'avant-garde.
                </p>
                <p>
                  Cette documentation interactive est conçue pour accompagner les membres de l'équipe et inspirer les nouveaux contributeurs à explorer et comprendre chaque facette innovante de notre projet technologique.
                </p>
              </div>
              
              <div className="mt-8 flex flex-wrap gap-4 justify-center">
                <span className="px-4 py-2 bg-blue-100 text-blue-700 rounded-full text-sm font-medium">Robotique</span>
                <span className="px-4 py-2 bg-emerald-100 text-emerald-700 rounded-full text-sm font-medium">Innovation</span>
                <span className="px-4 py-2 bg-purple-100 text-purple-700 rounded-full text-sm font-medium">Multidisciplinaire</span>
                <span className="px-4 py-2 bg-pink-100 text-pink-700 rounded-full text-sm font-medium">Open Source</span>
              </div>
            </div>
          </div>
        </section>

        {/* Cards Section */}
        <section className="mb-16 max-w-7xl mx-auto">
          <h2 className="text-4xl font-bold mb-12 text-center bg-gradient-to-r from-gray-800 to-gray-600 bg-clip-text text-transparent">
            Explorez nos domaines
          </h2>
          
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            {cardData.map((card) => {
              const IconComponent = card.icon;
              return (
                <div
                  key={card.title}
                  className="group relative bg-white/70 backdrop-blur-lg p-8 rounded-3xl shadow-xl border border-white/20 hover:shadow-2xl transition-all duration-100 hover:scale-105 hover:-translate-y-2"
                  data-aos="fade-up"
                  data-aos-delay={card.delay}
                >
                  {/* Gradient background on hover */}
                  <div className={`absolute inset-0 bg-gradient-to-br ${card.color} opacity-0 group-hover:opacity-10 rounded-3xl transition-opacity duration-100`}></div>
                  
                  {/* Icon with gradient background */}
                  <div className={`relative w-16 h-16 bg-gradient-to-br ${card.color} rounded-2xl flex items-center justify-center mb-6 group-hover:scale-110 transition-transform duration-100`}>
                    <IconComponent className="w-8 h-8 text-white" />
                  </div>
                  
                  <h3 className={`text-2xl font-bold mb-4 ${card.textColor} group-hover:scale-105 transition-transform duration-100`}>
                    {card.title}
                  </h3>
                  
                  <p className="text-gray-600 mb-6 leading-relaxed">
                    {card.description}
                  </p>
                  
                  <Link
                    to={card.href}
                    className={`inline-flex items-center gap-2 ${card.textColor} hover:gap-3 transition-all duration-100 font-semibold group-hover:scale-105`}
                  >
                    En savoir plus
                    <ChevronRight className="w-5 h-5 group-hover:translate-x-1 transition-transform duration-100" />
                  </Link>
                  
                  {/* Subtle border glow on hover */}
                  <div className={`absolute inset-0 bg-gradient-to-br ${card.color} opacity-0 group-hover:opacity-20 rounded-3xl blur-xl transition-opacity duration-100 -z-10`}></div>
                </div>
              );
            })}
          </div>
        </section>

        {/* Call to Action */}
        <section className="text-center max-w-4xl mx-auto" data-aos="fade-up" data-aos-delay="200">
          <div className="bg-gradient-to-r from-blue-600 to-purple-600 p-12 rounded-3xl text-white shadow-2xl">
            <h3 className="text-3xl font-bold mb-4">Prêt à plonger dans l'innovation ?</h3>
            <p className="text-xl mb-8 opacity-90">Rejoignez notre communauté et contribuez à façonner l'avenir de la robotique</p>
            <button className="bg-white text-blue-600 px-8 py-4 rounded-2xl font-bold text-lg hover:bg-gray-100 hover:scale-105 transition-all duration-100 shadow-lg">
              Commencer l'exploration
            </button>
          </div>
        </section>
      </div>
    </div>
  );
};

export default Home;