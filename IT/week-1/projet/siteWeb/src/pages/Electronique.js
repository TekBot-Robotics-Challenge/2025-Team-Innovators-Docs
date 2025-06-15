import { jsx as _jsx, jsxs as _jsxs } from "react/jsx-runtime";
import { useEffect, useState } from 'react';
import { Zap, Cpu, CircuitBoard, Settings, ChevronRight, FileText } from 'lucide-react';
import ReactMarkdown from 'react-markdown';
import readmeContent from '../../../../../Elec/semaine1.md?raw'; // Assurez-vous que le chemin est correct
const Electronique = () => {
    const [data, setData] = useState(null);
    const [activeSection, setActiveSection] = useState('');
    // Import des données depuis le fichier JSON
    useEffect(() => {
        // Charger les données depuis le fichier JSON
        import('../data/electronique.json')
            .then((jsonData) => {
            setData(jsonData.default);
        })
            .catch((error) => {
            console.error("Erreur lors du chargement des données électroniques:", error);
        });
    }, []);
    useEffect(() => {
        // Réinitialiser AOS pour les animations
        if (window.AOS) {
            window.AOS.refresh();
        }
        // Observer pour la navigation active
        const handleScroll = () => {
            if (!data)
                return;
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
    const scrollToSection = (sectionId) => {
        const element = document.getElementById(sectionId);
        if (element) {
            element.scrollIntoView({ behavior: 'smooth', block: 'start' });
        }
    };
    const getSectionIcon = (sectionId) => {
        const icons = {
            composants: CircuitBoard,
            circuits: Cpu,
            capteurs: Settings,
            programmation: Zap
        };
        return icons[sectionId] || FileText;
    };
    if (!data) {
        return (_jsx("div", { className: "min-h-screen bg-gradient-to-br from-blue-50 via-cyan-50 to-indigo-100 flex items-center justify-center", children: _jsxs("div", { className: "flex items-center gap-3 text-blue-600", children: [_jsx("div", { className: "animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600" }), _jsx("span", { className: "text-lg font-medium", children: "Chargement..." })] }) }));
    }
    return (_jsxs("div", { className: "min-h-screen bg-gradient-to-br from-blue-50 via-cyan-50 to-indigo-100 relative overflow-hidden", children: [_jsxs("div", { className: "absolute inset-0 overflow-hidden pointer-events-none", children: [_jsx("div", { className: "absolute -top-40 -right-40 w-96 h-96 bg-gradient-to-br from-blue-400/20 to-cyan-600/20 rounded-full blur-3xl" }), _jsx("div", { className: "absolute -bottom-40 -left-40 w-96 h-96 bg-gradient-to-tr from-indigo-400/20 to-blue-600/20 rounded-full blur-3xl" }), _jsx("div", { className: "absolute top-1/3 left-1/4 w-64 h-64 bg-gradient-to-r from-cyan-400/10 to-blue-400/10 rounded-full blur-2xl" })] }), _jsxs("div", { className: "relative z-10 w-full", children: [_jsx("section", { className: "px-6 py-16 max-w-7xl mx-auto", "data-aos": "fade-up", "data-aos-delay": "200", children: _jsxs("div", { className: "text-center mb-12", children: [_jsxs("div", { className: "inline-flex items-center gap-2 bg-gradient-to-r from-blue-600 to-cyan-600 bg-clip-text text-transparent mb-4", children: [_jsx(Zap, { className: "w-8 h-8 text-blue-500 motion-preset-oscillate motion-duration-700" }), _jsx("span", { className: "text-lg font-semibold uppercase tracking-wider", children: "\u00C9lectronique \u2022 Circuits \u2022 Innovation" })] }), _jsx("h1", { className: "text-5xl md:text-6xl font-black mb-6 bg-gradient-to-r from-gray-900 via-blue-800 to-cyan-800 bg-clip-text text-transparent leading-tight", children: data.title }), _jsx("p", { className: "text-xl md:text-2xl text-gray-600 max-w-4xl mx-auto leading-relaxed", children: data.description })] }) }), _jsx("div", { className: "px-6 pb-12", children: _jsxs("div", { className: "max-w-7xl mx-auto grid grid-cols-1 lg:grid-cols-4 gap-8", children: [_jsx("div", { className: "lg:col-span-1", children: _jsx("div", { className: "sticky top-6", children: _jsxs("div", { className: "bg-white/70 backdrop-blur-lg p-6 rounded-3xl shadow-xl border border-white/20", children: [_jsxs("h2", { className: "text-2xl font-bold mb-6 bg-gradient-to-r from-blue-700 to-cyan-700 bg-clip-text text-transparent flex items-center gap-2", children: [_jsx(FileText, { className: "w-6 h-6 text-blue-600" }), "Sections"] }), _jsx("nav", { className: "space-y-2", children: data.sections.map((section) => {
                                                        const IconComponent = getSectionIcon(section.id);
                                                        const isActive = activeSection === section.id;
                                                        return (_jsxs("button", { onClick: () => scrollToSection(section.id), className: `w-full text-left p-4 rounded-2xl transition-all duration-300 flex items-center gap-3 group ${isActive
                                                                ? 'bg-gradient-to-r from-blue-500 to-cyan-500 text-white shadow-lg scale-105'
                                                                : 'hover:bg-blue-50 text-gray-700 hover:scale-102'}`, children: [_jsx(IconComponent, { className: `w-5 h-5 ${isActive ? 'text-white' : 'text-blue-600'} transition-transform group-hover:scale-110` }), _jsx("span", { className: "font-medium", children: section.title }), _jsx(ChevronRight, { className: `w-4 h-4 ml-auto transition-transform ${isActive ? 'translate-x-1' : 'group-hover:translate-x-1'}` })] }, section.id));
                                                    }) })] }) }) }), _jsx("div", { className: "lg:col-span-3", children: _jsx("div", { className: "space-y-8", children: data.sections.map((section, index) => {
                                            const IconComponent = getSectionIcon(section.id);
                                            return (_jsxs("div", { id: section.id, className: "group bg-white/70 backdrop-blur-lg p-8 md:p-12 rounded-3xl shadow-xl border border-white/20 hover:shadow-2xl transition-all duration-500 hover:scale-[1.02]", "data-aos": "fade-up", "data-aos-delay": `${index * 100}`, children: [_jsxs("div", { className: "flex items-center gap-4 mb-6", children: [_jsx("div", { className: "w-12 h-12 bg-gradient-to-br from-blue-500 to-cyan-500 rounded-2xl flex items-center justify-center group-hover:scale-110 transition-transform duration-300", children: _jsx(IconComponent, { className: "w-6 h-6 text-white" }) }), _jsx("h2", { className: "text-3xl font-bold text-blue-700 group-hover:text-blue-600 transition-colors duration-300", children: section.title })] }), _jsxs("div", { className: "prose prose-lg max-w-none", children: [_jsx("p", { className: "text-gray-700 leading-relaxed text-lg", children: section.content }), _jsx("a", { href: section.link, className: " text-lg", children: "Voir plus" }), _jsx(ReactMarkdown, { children: readmeContent })] }), _jsx("div", { className: "mt-8 flex justify-end", children: _jsx("div", { className: "w-16 h-1 bg-gradient-to-r from-blue-400 to-cyan-400 rounded-full opacity-60 group-hover:opacity-100 transition-opacity duration-300" }) })] }, section.id));
                                        }) }) })] }) }), _jsx("section", { className: "px-6 py-16 max-w-4xl mx-auto", "data-aos": "fade-up", "data-aos-delay": "250", children: _jsxs("div", { className: "bg-gradient-to-r from-blue-600 to-cyan-600 p-12 rounded-3xl text-white shadow-2xl text-center", children: [_jsx(Zap, { className: "w-16 h-16 mx-auto mb-6 animate-pulse" }), _jsx("h3", { className: "text-3xl font-bold mb-4", children: "Pr\u00EAt \u00E0 explorer ?" }), _jsx("p", { className: "text-xl mb-8 opacity-90", children: "D\u00E9couvrez nos sch\u00E9mas d\u00E9taill\u00E9s et commencez votre projet \u00E9lectronique" }), _jsx("button", { className: "bg-white text-blue-600 px-8 py-4 rounded-2xl font-bold text-lg hover:bg-gray-100 hover:scale-105 transition-all duration-300 shadow-lg", children: "T\u00E9l\u00E9charger les sch\u00E9mas" })] }) })] })] }));
};
export default Electronique;
