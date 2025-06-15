import { jsx as _jsx, jsxs as _jsxs } from "react/jsx-runtime";
import { useEffect, useState } from 'react';
import { ChevronRight, Zap, Cog, Code, ArrowDown, Sparkles } from 'lucide-react';
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
    return (_jsxs("div", { className: "min-h-screen w-full bg-gradient-to-br from-slate-50 via-blue-50 to-indigo-100 relative overflow-hidden pt-20", children: [_jsxs("div", { className: "absolute inset-0 overflow-hidden pointer-events-none", children: [_jsx("div", { className: "absolute -top-40 -right-40 w-96 h-96 bg-gradient-to-br from-blue-400/20 to-purple-600/20 rounded-full blur-3xl" }), _jsx("div", { className: "absolute -bottom-40 -left-40 w-96 h-96 bg-gradient-to-tr from-emerald-400/20 to-cyan-600/20 rounded-full blur-3xl" }), _jsx("div", { className: "absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-64 h-64 bg-gradient-to-r from-pink-400/10 to-yellow-400/10 rounded-full blur-2xl" })] }), _jsxs("div", { className: "relative z-10 w-full px-6 py-12", children: [_jsx("section", { className: "text-center mb-20 max-w-6xl mx-auto", children: _jsxs("div", { className: `transform transition-all duration-1000 ${isVisible ? 'translate-y-0 opacity-100' : 'translate-y-10 opacity-0'}`, children: [_jsxs("div", { className: "inline-flex items-center gap-2 bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent mb-4", children: [_jsx(Sparkles, { className: "w-6 h-6 text-blue-300 motion-preset-blink motion-duration-1000" }), _jsx("span", { className: "text-sm font-semibold uppercase tracking-wider", children: "Innovation \u2022 Technologie \u2022 Robotique" })] }), _jsxs("h1", { className: "text-6xl md:text-7xl font-black mb-6 bg-gradient-to-r from-gray-900 via-blue-800 to-purple-800 bg-clip-text text-transparent leading-tight", children: ["Innovator-TRC", _jsx("span", { className: "block text-4xl md:text-5xl font-light mt-2", children: "Documentation" })] }), _jsx("p", { className: "text-xl md:text-2xl text-gray-600 max-w-4xl mx-auto leading-relaxed mb-8", children: "Bienvenue dans la documentation compl\u00E8te du projet TechBot. Explorez les diff\u00E9rentes sections pour d\u00E9couvrir les aspects \u00E9lectroniques, m\u00E9caniques et informatiques qui donnent vie \u00E0 l'innovation." }), _jsx("div", { className: "animate-bounce mt-12", children: _jsx(ArrowDown, { className: "w-8 h-8 text-gray-400 mx-auto" }) })] }) }), _jsx("section", { className: "mb-20 max-w-6xl mx-auto", "data-aos": "fade-up", "data-aos-delay": "200", children: _jsxs("div", { className: "max-w-4xl mx-auto", children: [_jsx("h2", { className: "text-4xl md:text-5xl font-bold mb-8 text-center bg-gradient-to-r from-gray-800 to-gray-600 bg-clip-text text-transparent", children: "\u00C0 propos du projet" }), _jsxs("div", { className: "bg-white/70 backdrop-blur-lg p-8 md:p-12 rounded-3xl shadow-2xl border border-white/20 hover:shadow-3xl transition-all duration-100 hover:scale-[1.02]", children: [_jsxs("div", { className: "space-y-6 text-lg text-gray-700 leading-relaxed", children: [_jsx("p", { className: "text-xl font-medium text-gray-800", children: "Le projet TechBot est une initiative multidisciplinaire r\u00E9volutionnaire qui fusionne harmonieusement \u00E9lectronique, m\u00E9canique et informatique pour cr\u00E9er une solution robotique d'avant-garde." }), _jsx("p", { children: "Cette documentation interactive est con\u00E7ue pour accompagner les membres de l'\u00E9quipe et inspirer les nouveaux contributeurs \u00E0 explorer et comprendre chaque facette innovante de notre projet technologique." })] }), _jsxs("div", { className: "mt-8 flex flex-wrap gap-4 justify-center", children: [_jsx("span", { className: "px-4 py-2 bg-blue-100 text-blue-700 rounded-full text-sm font-medium", children: "Robotique" }), _jsx("span", { className: "px-4 py-2 bg-emerald-100 text-emerald-700 rounded-full text-sm font-medium", children: "Innovation" }), _jsx("span", { className: "px-4 py-2 bg-purple-100 text-purple-700 rounded-full text-sm font-medium", children: "Multidisciplinaire" }), _jsx("span", { className: "px-4 py-2 bg-pink-100 text-pink-700 rounded-full text-sm font-medium", children: "Open Source" })] })] })] }) }), _jsxs("section", { className: "mb-16 max-w-7xl mx-auto", children: [_jsx("h2", { className: "text-4xl font-bold mb-12 text-center bg-gradient-to-r from-gray-800 to-gray-600 bg-clip-text text-transparent", children: "Explorez nos domaines" }), _jsx("div", { className: "grid grid-cols-1 md:grid-cols-3 gap-8", children: cardData.map((card) => {
                                    const IconComponent = card.icon;
                                    return (_jsxs("div", { className: "group relative bg-white/70 backdrop-blur-lg p-8 rounded-3xl shadow-xl border border-white/20 hover:shadow-2xl transition-all duration-100 hover:scale-105 hover:-translate-y-2", "data-aos": "fade-up", "data-aos-delay": card.delay, children: [_jsx("div", { className: `absolute inset-0 bg-gradient-to-br ${card.color} opacity-0 group-hover:opacity-10 rounded-3xl transition-opacity duration-100` }), _jsx("div", { className: `relative w-16 h-16 bg-gradient-to-br ${card.color} rounded-2xl flex items-center justify-center mb-6 group-hover:scale-110 transition-transform duration-100`, children: _jsx(IconComponent, { className: "w-8 h-8 text-white" }) }), _jsx("h3", { className: `text-2xl font-bold mb-4 ${card.textColor} group-hover:scale-105 transition-transform duration-100`, children: card.title }), _jsx("p", { className: "text-gray-600 mb-6 leading-relaxed", children: card.description }), _jsxs("a", { href: card.href, className: `inline-flex items-center gap-2 ${card.textColor} hover:gap-3 transition-all duration-100 font-semibold group-hover:scale-105`, children: ["En savoir plus", _jsx(ChevronRight, { className: "w-5 h-5 group-hover:translate-x-1 transition-transform duration-100" })] }), _jsx("div", { className: `absolute inset-0 bg-gradient-to-br ${card.color} opacity-0 group-hover:opacity-20 rounded-3xl blur-xl transition-opacity duration-100 -z-10` })] }, card.title));
                                }) })] }), _jsx("section", { className: "text-center max-w-4xl mx-auto", "data-aos": "fade-up", "data-aos-delay": "200", children: _jsxs("div", { className: "bg-gradient-to-r from-blue-600 to-purple-600 p-12 rounded-3xl text-white shadow-2xl", children: [_jsx("h3", { className: "text-3xl font-bold mb-4", children: "Pr\u00EAt \u00E0 plonger dans l'innovation ?" }), _jsx("p", { className: "text-xl mb-8 opacity-90", children: "Rejoignez notre communaut\u00E9 et contribuez \u00E0 fa\u00E7onner l'avenir de la robotique" }), _jsx("button", { className: "bg-white text-blue-600 px-8 py-4 rounded-2xl font-bold text-lg hover:bg-gray-100 hover:scale-105 transition-all duration-100 shadow-lg", children: "Commencer l'exploration" })] }) })] })] }));
};
export default Home;
