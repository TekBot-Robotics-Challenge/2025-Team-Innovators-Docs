import { useState, useEffect } from 'react';
import { Menu, X, Zap, Cog, Code, Home, ChevronDown } from 'lucide-react';
import { Link } from 'react-router-dom';
import logo from '/logo.png';

const Navbar = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [scrolled, setScrolled] = useState(false);
  const [visible, setVisible] = useState(true);
  const [lastScrollY, setLastScrollY] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      const currentScrollY = window.scrollY;
      
      // Détermine si le navbar doit être visible
      if (currentScrollY < lastScrollY || currentScrollY < 10) {
        // Scroll vers le haut ou proche du haut de la page
        setVisible(true);
      } else if (currentScrollY > lastScrollY && currentScrollY > 100) {
        // Scroll vers le bas et assez loin de la page
        setVisible(false);
        setIsOpen(false); // Ferme le menu mobile si ouvert
      }
      
      // Style de background basé sur la position de scroll
      setScrolled(currentScrollY > 50);
      setLastScrollY(currentScrollY);
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    return () => window.removeEventListener('scroll', handleScroll);
  }, [lastScrollY]);

  const navigation = [
    {
      name: 'Accueil',
      href: '/',
      icon: Home,
      color: 'from-blue-500 to-purple-500'
    },
    {
      name: 'Électronique',
      href: '/electronique',
      icon: Zap,
      color: 'from-blue-500 to-cyan-500'
    },
    {
      name: 'Mécanique',
      href: '/mecanique',
      icon: Cog,
      color: 'from-emerald-500 to-teal-500'
    },
    {
      name: 'IT',
      href: '/it',
      icon: Code,
      color: 'from-purple-500 to-pink-500'
    }
  ];

  return (
    <nav className={`fixed top-0 w-full z-50 transition-all duration-300 ${
      visible ? 'translate-y-0' : '-translate-y-full'
    } ${
      scrolled 
        ? 'bg-gray-900/95 backdrop-blur-lg shadow-2xl border-b border-gray-700/50' 
        : 'bg-gray-800/90 backdrop-blur-sm'
    }`}>
      <div className="w-full px-6 py-4">
        <div className="flex justify-between items-center">
          {/* Logo */}
          <div className="flex items-center gap-3">
              <img src={logo} className="w-10 h-10 text-white rounded-lg flex items-center justify-center" />
            <div>
              <h1 className="text-xl font-bold bg-gradient-to-r from-white to-gray-300 bg-clip-text text-transparent">
                Innovator Documentation
              </h1>
              <p className="text-xs text-gray-400 hidden sm:block">Innovation • Technologie • Robotique</p>
            </div>
          </div>

          {/* Desktop Navigation */}
          <div className="hidden lg:flex items-center space-x-2">
            {navigation.map((item) => {
              const IconComponent = item.icon;
              return (
                <Link
                  key={item.name}
                  to={item.href}
                  className="group relative px-6 py-3 rounded-2xl transition-all duration-300 hover:scale-105"
                >
                  {/* Gradient background on hover */}
                  <div className={`absolute inset-0 bg-gradient-to-r ${item.color} opacity-0 group-hover:opacity-20 rounded-2xl transition-opacity duration-300`}></div>
                  
                  <div className="relative flex items-center gap-2">
                    <IconComponent className="w-5 h-5 text-gray-300 group-hover:text-white transition-colors duration-300" />
                    <span className="text-gray-300 group-hover:text-white font-medium transition-colors duration-300">
                      {item.name}
                    </span>
                  </div>
                  
                  {/* Animated underline */}
                  <div className={`absolute bottom-0 left-1/2 transform -translate-x-1/2 w-0 h-0.5 bg-gradient-to-r ${item.color} group-hover:w-3/4 transition-all duration-300 rounded-full`}></div>
                </Link>
              );
            })}
          </div>

          {/* Mobile menu button */}
          <button
            onClick={() => setIsOpen(!isOpen)}
            className="lg:hidden p-2 rounded-xl bg-gray-700/50 hover:bg-gray-600/50 transition-colors duration-300"
          >
            {isOpen ? (
              <X className="w-6 h-6 text-white" />
            ) : (
              <Menu className="w-6 h-6 text-white" />
            )}
          </button>
        </div>

        {/* Mobile Navigation */}
        <div className={`lg:hidden transition-all duration-300 overflow-hidden ${
          isOpen ? 'max-h-96 opacity-100 mt-6' : 'max-h-0 opacity-0'
        }`}>
          <div className="bg-gray-800/90 backdrop-blur-lg rounded-2xl p-4 space-y-2">
            {navigation.map((item, index) => {
              const IconComponent = item.icon;
              return (
                <Link
                  key={item.name}
                  to
                  ={item.href}
                  onClick={() => setIsOpen(false)}
                  className="group flex items-center gap-3 p-4 rounded-xl hover:bg-gray-700/50 transition-all duration-300"
                  style={{ animationDelay: `${index * 100}ms` }}
                >
                  <div className={`w-8 h-8 bg-gradient-to-r ${item.color} rounded-lg flex items-center justify-center`}>
                    <IconComponent className="w-4 h-4 text-white" />
                  </div>
                  <span className="text-gray-300 group-hover:text-white font-medium transition-colors duration-300">
                    {item.name}
                  </span>
                  <ChevronDown className="w-4 h-4 text-gray-400 ml-auto transform -rotate-90 group-hover:translate-x-1 transition-transform duration-300" />
                </Link>
              );
            })}
          </div>
        </div>
      </div>
    </nav>
  );
};

export default Navbar;