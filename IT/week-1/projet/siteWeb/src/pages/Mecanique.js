import { jsx as _jsx, jsxs as _jsxs } from "react/jsx-runtime";
import { useEffect, useState } from 'react';
import { Cog, Settings, Wrench, Cpu, ChevronRight, FileText } from 'lucide-react';
const Mecanique = () => {
    const [data, setData] = useState(null);
    const [activeSection, setActiveSection] = useState('');
    // Import des données depuis le fichier JSON
    useEffect(() => {
        // Charger les données depuis le fichier JSON
        import('../data/mecanique.json')
            .then((jsonData) => {
            setData(jsonData.default);
        })
            .catch((error) => {
            console.error("Erreur lors du chargement des données mécaniques:", error);
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
            composants: Cog,
            assemblage: Settings,
            materiaux: Wrench,
            maintenance: Cpu
        };
        return icons[sectionId] || FileText;
    };
    if (!data) {
        return (_jsx("div", { className: "min-h-screen bg-gradient-to-br from-emerald-50 via-teal-50 to-green-100 flex items-center justify-center", children: _jsxs("div", { className: "flex items-center gap-3 text-emerald-600", children: [_jsx("div", { className: "animate-spin rounded-full h-8 w-8 border-b-2 border-emerald-600" }), _jsx("span", { className: "text-lg font-medium", children: "Chargement..." })] }) }));
    }
    return (_jsxs("div", { className: "min-h-screen bg-gradient-to-br from-emerald-50 via-teal-50 to-green-100 relative overflow-hidden", children: [_jsxs("div", { className: "absolute inset-0 overflow-hidden pointer-events-none", children: [_jsx("div", { className: "absolute -top-40 -right-40 w-96 h-96 bg-gradient-to-br from-emerald-400/20 to-teal-600/20 rounded-full blur-3xl" }), _jsx("div", { className: "absolute -bottom-40 -left-40 w-96 h-96 bg-gradient-to-tr from-green-400/20 to-emerald-600/20 rounded-full blur-3xl" }), _jsx("div", { className: "absolute top-1/3 left-1/4 w-64 h-64 bg-gradient-to-r from-teal-400/10 to-green-400/10 rounded-full blur-2xl" })] }), _jsxs("div", { className: "relative z-10 w-full", children: [_jsx("section", { className: "px-6 py-16 max-w-7xl mx-auto", "data-aos": "fade-up", "data-aos-delay": "200", children: _jsxs("div", { className: "text-center mb-12", children: [_jsxs("div", { className: "inline-flex items-center gap-2 bg-gradient-to-r from-emerald-600 to-teal-600 bg-clip-text text-transparent mb-4", children: [_jsx(Cog, { className: "w-8 h-8 text-emerald-500 motion-preset-spin" }), _jsx("span", { className: "text-lg font-semibold uppercase tracking-wider", children: "M\u00E9canique \u2022 Conception \u2022 Innovation" })] }), _jsx("h1", { className: "text-5xl md:text-6xl font-black mb-6 bg-gradient-to-r from-gray-900 via-emerald-800 to-teal-800 bg-clip-text text-transparent leading-tight", children: data.title }), _jsx("p", { className: "text-xl md:text-2xl text-gray-600 max-w-4xl mx-auto leading-relaxed", children: data.description })] }) }), _jsx("div", { className: "px-6 pb-12", children: _jsxs("div", { className: "max-w-7xl mx-auto grid grid-cols-1 lg:grid-cols-4 gap-8", children: [_jsx("div", { className: "lg:col-span-1", children: _jsx("div", { className: "sticky top-6", children: _jsxs("div", { className: "bg-white/70 backdrop-blur-lg p-6 rounded-3xl shadow-xl border border-white/20", children: [_jsxs("h2", { className: "text-2xl font-bold mb-6 bg-gradient-to-r from-emerald-700 to-teal-700 bg-clip-text text-transparent flex items-center gap-2", children: [_jsx(FileText, { className: "w-6 h-6 text-emerald-600" }), "Sections"] }), _jsx("nav", { className: "space-y-2", children: data.sections.map((section) => {
                                                        const IconComponent = getSectionIcon(section.id);
                                                        const isActive = activeSection === section.id;
                                                        return (_jsxs("button", { onClick: () => scrollToSection(section.id), className: `w-full text-left p-4 rounded-2xl transition-all duration-300 flex items-center gap-3 group ${isActive
                                                                ? 'bg-gradient-to-r from-emerald-500 to-teal-500 text-white shadow-lg scale-105'
                                                                : 'hover:bg-emerald-50 text-gray-700 hover:scale-102'}`, children: [_jsx(IconComponent, { className: `w-5 h-5 ${isActive ? 'text-white' : 'text-emerald-600'} transition-transform group-hover:scale-110` }), _jsx("span", { className: "font-medium", children: section.title }), _jsx(ChevronRight, { className: `w-4 h-4 ml-auto transition-transform ${isActive ? 'translate-x-1' : 'group-hover:translate-x-1'}` })] }, section.id));
                                                    }) })] }) }) }), _jsx("div", { className: "lg:col-span-3", children: _jsx("div", { className: "space-y-8", children: data.sections.map((section, index) => {
                                            const IconComponent = getSectionIcon(section.id);
                                            return (_jsxs("div", { id: section.id, className: "group bg-white/70 backdrop-blur-lg p-8 md:p-12 rounded-3xl shadow-xl border border-white/20 hover:shadow-2xl transition-all duration-500 hover:scale-[1.02]", "data-aos": "fade-up", "data-aos-delay": `${index * 100}`, children: [_jsxs("div", { className: "flex items-center gap-4 mb-6", children: [_jsx("div", { className: "w-12 h-12 bg-gradient-to-br from-emerald-500 to-teal-500 rounded-2xl flex items-center justify-center group-hover:scale-110 transition-transform duration-300", children: _jsx(IconComponent, { className: "w-6 h-6 text-white" }) }), _jsx("h2", { className: "text-3xl font-bold text-emerald-700 group-hover:text-emerald-600 transition-colors duration-300", children: section.title })] }), _jsxs("div", { className: "prose prose-lg max-w-none", children: [_jsx("p", { className: "text-gray-700 leading-relaxed text-lg", children: section.content }), _jsx("p", { className: "text-gray-700 leading-relaxed text-lg", children: section.link })] }), _jsx("div", { className: "mt-8 flex justify-end", children: _jsx("div", { className: "w-16 h-1 bg-gradient-to-r from-emerald-400 to-teal-400 rounded-full opacity-60 group-hover:opacity-100 transition-opacity duration-300" }) })] }, section.id));
                                        }) }) })] }) }), _jsx("section", { className: "px-6 py-16 max-w-4xl mx-auto", "data-aos": "fade-up", "data-aos-delay": "250", children: _jsxs("div", { className: "bg-gradient-to-r from-emerald-600 to-teal-600 p-12 rounded-3xl text-white shadow-2xl text-center", children: [_jsx(Cog, { className: "w-16 h-16 mx-auto mb-6 animate-spin", style: { animationDuration: '3s' } }), _jsx("h3", { className: "text-3xl font-bold mb-4", children: "Pr\u00EAt \u00E0 construire ?" }), _jsx("p", { className: "text-xl mb-8 opacity-90", children: "D\u00E9couvrez nos guides d\u00E9taill\u00E9s et commencez votre projet m\u00E9canique" }), _jsx("button", { className: "bg-white text-emerald-600 px-8 py-4 rounded-2xl font-bold text-lg hover:bg-gray-100 hover:scale-105 transition-all duration-300 shadow-lg", children: "T\u00E9l\u00E9charger les plans" })] }) })] })] }));
};
export default Mecanique;
