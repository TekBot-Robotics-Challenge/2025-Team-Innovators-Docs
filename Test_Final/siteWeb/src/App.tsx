import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import { useEffect } from "react";
import AOS from "aos";
import "aos/dist/aos.css";
import "./App.css";

// Pages
import Home from "./pages/Collecte";

const basename = import.meta.env.DEV
  ? "/" // en dev local, pas de base spécifique
  : "/2025-Team-Innovators-Triage-Déchets"; // en prod GitHub Pages
function App() {
  console.log(
    import.meta.env.DEV
      ? "Environnement de développement"
      : "Environnement de production"
  );
  useEffect(() => {
    AOS.init({
      duration: 1000,
      once: false,
    });
  }, []);

  return (
    // <Router basename="/2025-Team-Innovators-Docs">
    <Router basename={basename}>
      <div className="min-h-screen w-full">
        
          <Routes>
            <Route path="/" element={<Home />} /> 
          </Routes>
      </div>
    </Router>
  );
}

export default App;
