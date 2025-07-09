import React, { useState } from 'react';
import { Trash2, Recycle, Leaf, Zap } from 'lucide-react';

interface WasteCard {
  title: string;
  count: number;
  icon: React.ReactNode;
  color: string;
  bgColor: string;
}

const Collecte: React.FC = () => {
  // État pour les compteurs des déchets
  const [wasteCounts, setWasteCounts] = useState({
    recyclable: 0,
    organic: 0,
    electronic: 0,
    residual: 0
  });

  // État pour suivre la carte cliquée
  const [clickedCard, setClickedCard] = useState<string | null>(null);

  // Calculer le total
  const totalWaste = wasteCounts.recyclable + wasteCounts.organic + wasteCounts.electronic + wasteCounts.residual;

  // Fonction pour incrémenter les compteurs
  const handleCardClick = (cardType: string) => {
    setWasteCounts(prev => ({
      ...prev,
      [cardType]: prev[cardType] + 1
    }));
    
    // Mettre à jour la carte cliquée et retirer l'effet après 200ms
    setClickedCard(cardType);
    setTimeout(() => {
      setClickedCard(null);
    }, 200);
  };

  // Appliquer les styles au body
  React.useEffect(() => {
    document.body.style.margin = '0';
    document.body.style.placeItems = 'center';
    // Supprimer les propriétés indésirables
    document.body.style.display = '';
    document.body.style.minWidth = '';
    document.body.style.minHeight = '';
  }, []);
  
  const wasteCategories: WasteCard[] = [
    {
      title: "Déchets recyclables",
      count: wasteCounts.recyclable,
      icon: <Recycle className="w-8 h-8" />,
      color: "text-green-600",
      bgColor: "bg-green-50"
    },
    {
      title: "Déchets organiques",
      count: wasteCounts.organic,
      icon: <Leaf className="w-8 h-8" />,
      color: "text-yellow-600",
      bgColor: "bg-yellow-50"
    },
    {
      title: "Déchets électroniques",
      count: wasteCounts.electronic,
      icon: <Zap className="w-8 h-8" />,
      color: "text-blue-600",
      bgColor: "bg-blue-50"
    },
    {
      title: "Déchets résiduels",
      count: wasteCounts.residual,
      icon: <Trash2 className="w-8 h-8" />,
      color: "text-red-600",
      bgColor: "bg-red-50"
    }
  ];

  // Fonction pour obtenir la couleur de fond plus foncée
  const getDarkerBgColor = (cardType: string) => {
    const darkColors = {
      recyclable: "bg-green-200",
      organic: "bg-yellow-200", 
      electronic: "bg-blue-200",
      residual: "bg-red-200"
    };
    return darkColors[cardType] || "bg-gray-200";
  };

  return (
    <div className="min-h-screen bg-white p-8">
      <div className="w-full">
        {/* Header avec logos */}
        <div className="flex justify-between items-center mb-12">
          <div className="flex items-center space-x-3">
            <div className="w-12 h-12 bg-green-100 rounded-full flex items-center justify-center">
               <img src="" alt="Teckbot Robotics" />
            </div>
          </div>
          
          <div className="flex items-center space-x-3">
            <div className="w-25 h-25  rounded-full flex items-center justify-center">
              <img src="/image/logo_innovator.jpeg" alt="Innovators" />
            </div>
          </div>
        </div>

        {/* Titre principal */}
        <div className="text-center mb-12">
          <h3 className="text-3xl font-bold text-gray-800 mb-2">
            Statistiques de Tri des Déchets
          </h3>
          <p className="text-gray-600 text-lg">
            Suivi en temps réel de la collecte et du tri du recyclage de déchets du groupe Innovators
          </p>
        </div>

        {/* Carte principale - Total */}
        <div className="mb-12">
          <div className="bg-gradient-to-r from-black to-gray-600 rounded-2xl p-8 text-white shadow-lg">
            <div className="flex items-center justify-between">
              <div>
                <h2 className="text-2xl font-semibold mb-2">Total des déchets triés</h2>
                <p className="text-4xl font-bold">{totalWaste.toLocaleString()}</p>
                <p className="text-gray-300 mt-2">unités collectées instantanément</p>
              </div>
              <div className="bg-white/20 rounded-full p-6">
                <Trash2 className="w-16 h-16 text-white" />
              </div>
            </div>
          </div>
        </div>

        {/* Cartes des catégories */}
        <div>
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
            {wasteCategories.map((category, index) => {
              const cardTypes = ['recyclable', 'organic', 'electronic', 'residual'];
              const cardType = cardTypes[index];
              
              return (
                <div
                  key={index}
                  className={`${clickedCard === cardType ? getDarkerBgColor(cardType) : category.bgColor} rounded-xl p-6 border border-gray-100 shadow-sm hover:shadow-md transition-all duration-200 cursor-pointer transform hover:scale-105`}
                  onClick={() => handleCardClick(cardType)}
                >
                  <div className="flex items-center justify-between mb-4">
                    <div className={`${category.color} p-3 rounded-lg bg-white`}>
                      {category.icon}
                    </div>
                    <div className="text-right">
                      <p className={`text-2xl font-bold ${category.color}`}>
                        {category.count.toLocaleString()}
                      </p>
                      <p className="text-sm text-gray-500">
                        {totalWaste > 0 ? ((category.count / totalWaste) * 100).toFixed(1) : 0}%
                      </p>
                    </div>
                  </div>
                  <h3 className="font-semibold text-gray-800 text-sm">
                    {category.title}
                  </h3>
                  <div className="mt-3 bg-gray-200 rounded-full h-2">
                    <div
                      className={`h-2 rounded-full ${category.color.replace('text-', 'bg-')}`}
                      style={{ width: totalWaste > 0 ? `${(category.count / totalWaste) * 100}%` : '0%' }}
                    ></div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Footer */}
        <div className="text-center mt-12 text-gray-500">
          <p>Dernière mise à jour: {new Date().toLocaleDateString('fr-FR')}</p>
        </div>
      </div>
    </div>
  );
};

export default Collecte;