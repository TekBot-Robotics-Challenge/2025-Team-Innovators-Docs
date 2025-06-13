import { jsx as _jsx, jsxs as _jsxs } from "react/jsx-runtime";
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { useEffect } from 'react';
import AOS from 'aos';
import 'aos/dist/aos.css';
import './App.css';
// Components
import Navbar from './components/Navbar';
// Pages
import Home from './pages/Home';
import Electronique from './pages/Electronique';
import Mecanique from './pages/Mecanique';
import IT from './pages/IT';
function App() {
    useEffect(() => {
        AOS.init({
            duration: 1000,
            once: false,
        });
    }, []);
    return (_jsx(Router, { children: _jsxs("div", { className: "min-h-screen w-full", children: [_jsx(Navbar, {}), _jsx("main", { className: "pt-16 md:pt-20 w-full", children: _jsxs(Routes, { children: [_jsx(Route, { path: "/", element: _jsx(Home, {}) }), _jsx(Route, { path: "/electronique", element: _jsx(Electronique, {}) }), _jsx(Route, { path: "/mecanique", element: _jsx(Mecanique, {}) }), _jsx(Route, { path: "/it", element: _jsx(IT, {}) })] }) })] }) }));
}
export default App;
