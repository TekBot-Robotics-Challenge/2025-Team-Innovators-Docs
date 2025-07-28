import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import { useEffect } from "react";
import AOS from "aos";
import "aos/dist/aos.css";
import "./App.css";

// Components
import Navbar from "./components/Navbar";

// Pages
import Home from "./pages/Home";
import Electronique from "./pages/Electronique/Electronique";
import Mecanique from "./pages/Mecanique/Mecanique";
import IT from "./pages/It/IT";
import Convoyeur from "./pages/It/Convoyeur";
const basename = import.meta.env.DEV
  ? "/" // en dev local, pas de base spécifique
  : "/2025-Team-Innovators-Docs"; // en prod GitHub Pages
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
        <Navbar />
        <main className="pt-16 md:pt-20 w-full">
          <Routes>
            <Route path="/" element={<Home />} />
            <Route path="/electronique" element={<Electronique />} />
            <Route path="/mecanique" element={<Mecanique />} />
            <Route path="/it" element={<IT />} />
            <Route path="/convoyeur" element={<Convoyeur />} />
          </Routes>
        </main>
      </div>
    </Router>
  );
}

export default App;
