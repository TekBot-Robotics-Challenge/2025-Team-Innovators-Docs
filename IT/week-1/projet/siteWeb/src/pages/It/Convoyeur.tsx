import React, { useState, useEffect } from 'react';
import {
  Trash2,
  Recycle,
  Leaf,
  Zap,
  TrendingUp,
  Activity,
  Clock,
  BarChart3,
  Target,
  Award,
  RefreshCw,
  ChevronRight,
  CheckCircle2,
  HardDrive,
  Gauge,
  Database
} from 'lucide-react';
import { io, Socket } from 'socket.io-client';

interface WasteCard {
  title: string;
  count: number;
  icon: React.ReactNode;
  color: string;
  bgColor: string;
  darkBgColor: string;
  type: string;
  description: string;
}

interface FloatingPlusOne {
  id: number;
  type: string;
  top: number;
  left: number;
  opacity: number;
}

const ConvoyeurDashboard: React.FC = () => {
  const [wasteCounts, setWasteCounts] = useState({
    recyclable: 0,
    organic: 0,
    electronic: 0,
    residual: 0
  });

  const [clickedCard, setClickedCard] = useState<string | null>(null);
  const [totalProcessed, setTotalProcessed] = useState(0);
  const [efficiency, setEfficiency] = useState(0);
  const [socket, setSocket] = useState<Socket | null>(null);
  const [floatingPlusOnes, setFloatingPlusOnes] = useState<FloatingPlusOne[]>([]);

  const totalWaste = wasteCounts.recyclable + wasteCounts.organic + wasteCounts.electronic + wasteCounts.residual;

  // Initialiser la connexion Socket.io
  useEffect(() => {
    const newSocket = io('https://color-server-lbrd.onrender.com');
    setSocket(newSocket);

    return () => {
      newSocket.disconnect();
    };
  }, []);

  // Écouter les événements du serveur
  useEffect(() => {
    if (!socket) return;

    const handleWasteDetected = (data: { wasteType: string }) => {
      setWasteCounts(prev => ({
        ...prev,
        [data.wasteType as keyof typeof prev]: prev[data.wasteType as keyof typeof prev] + 1
      }));

      setClickedCard(data.wasteType);
      setTimeout(() => {
        setClickedCard(null);
      }, 300);
    };

    socket.on('waste_detected', handleWasteDetected);

    return () => {
      socket.off('waste_detected', handleWasteDetected);
    };
  }, [socket]);

  // Animation du compteur total
  useEffect(() => {
    if (totalWaste > totalProcessed) {
      const timer = setTimeout(() => {
        setTotalProcessed(totalWaste);
      }, 100);
      return () => clearTimeout(timer);
    }
  }, [totalWaste, totalProcessed]);

  // Calcul de l'efficacité
  useEffect(() => {
    const recyclableRate = totalWaste > 0 ? (wasteCounts.recyclable / totalWaste) * 100 : 0;
    const organicRate = totalWaste > 0 ? (wasteCounts.organic / totalWaste) * 100 : 0;
    const electronicRate = totalWaste > 0 ? (wasteCounts.electronic / totalWaste) * 100 : 0;
    const residualRate = totalWaste > 0 ? (wasteCounts.residual / totalWaste) * 100 : 0;
    setEfficiency(recyclableRate + organicRate + electronicRate + residualRate);
  }, [wasteCounts, totalWaste]);

  // Gérer les animations +1
  useEffect(() => {
    if (clickedCard) {
      const cardElement = document.getElementById(`card-${clickedCard}`);
      if (cardElement) {
        const rect = cardElement.getBoundingClientRect();
        const left = Math.random() * (rect.width - 40) + 20;
        const top = rect.height / 2;

        const newPlusOne: FloatingPlusOne = {
          id: Date.now(),
          type: clickedCard,
          top: top,
          left: left,
          opacity: 1
        };

        setFloatingPlusOnes(prev => [...prev, newPlusOne]);

        const fadeInterval = setInterval(() => {
          setFloatingPlusOnes(prev =>
            prev.map(po =>
              po.id === newPlusOne.id
                ? { ...po, opacity: po.opacity - 0.05, top: po.top - 2 }
                : po
            ).filter(po => po.opacity > 0)
          );
        }, 50);

        setTimeout(() => {
          clearInterval(fadeInterval);
          setFloatingPlusOnes(prev => prev.filter(po => po.id !== newPlusOne.id));
        }, 1000);
      }
    }
  }, [clickedCard]);

  const resetCounters = () => {
    setWasteCounts({
      recyclable: 0,
      organic: 0,
      electronic: 0,
      residual: 0
    });
    setTotalProcessed(0);
    setFloatingPlusOnes([]);
  };

  const wasteCategories: WasteCard[] = [
    {
      title: "Recyclables",
      count: wasteCounts.recyclable,
      icon: <Recycle className="w-6 h-6" />,
      color: "text-emerald-600",
      bgColor: "bg-emerald-50",
      darkBgColor: "bg-emerald-100",
      type: "recyclable",
      description: "Plastique, carton, métal"
    },
    {
      title: "Organiques",
      count: wasteCounts.organic,
      icon: <Leaf className="w-6 h-6" />,
      color: "text-amber-500",
      bgColor: "bg-amber-50",
      darkBgColor: "bg-amber-100",
      type: "organic",
      description: "Compost, biodégradable"
    },
    {
      title: "Électroniques",
      count: wasteCounts.electronic,
      icon: <Zap className="w-6 h-6" />,
      color: "text-blue-600",
      bgColor: "bg-blue-50",
      darkBgColor: "bg-blue-100",
      type: "electronic",
      description: "Appareils, batteries"
    },
    {
      title: "Résiduels",
      count: wasteCounts.residual,
      icon: <Trash2 className="w-6 h-6" />,
      color: "text-red-500",
      bgColor: "bg-red-50",
      darkBgColor: "bg-red-100",
      type: "residual",
      description: "Non recyclable"
    }
  ];

  const StatCard = ({
    title,
    value,
    icon,
    color,
    suffix = "",
    trend = null
  }: {
    title: string;
    value: string | number;
    icon: React.ReactNode;
    color: string;
    suffix?: string;
    trend?: 'up' | 'down' | null;
  }) => (
    <div className="bg-white rounded-xl p-5 shadow-sm border border-gray-100 hover:shadow-md transition-shadow">
      <div className="flex items-start justify-between">
        <div>
          <p className="text-sm font-medium text-gray-500 mb-1">{title}</p>
          <div className="text-2xl font-bold text-gray-800">
            {value}{suffix}
          </div>
        </div>
        <div className={`p-3 rounded-lg ${color} bg-opacity-10`}>
          {icon}
        </div>
      </div>
      {trend && (
        <div className={`mt-3 text-xs font-medium flex items-center ${trend === 'up' ? 'text-green-600' : 'text-red-600'}`}>
          {trend === 'up' ? (
            <TrendingUp className="w-3 h-3 mr-1" />
          ) : (
            <TrendingUp className="w-3 h-3 mr-1 transform rotate-180" />
          )}
          12% vs hier
        </div>
      )}
    </div>
  );

  const ProgressBar = ({
    percentage = 0,
    color = "",
    height = "h-2",
    showLabel = false,
    label = "",
    animated = true
  }) => (
    <div className="w-full">
      {showLabel && (
        <div className="flex justify-between items-center mb-1">
          <span className="text-xs font-medium text-gray-600">{label}</span>
          <span className="text-xs font-bold text-gray-800">{percentage.toFixed(1)}%</span>
        </div>
      )}
      <div className={`bg-gray-100 rounded-full ${height} overflow-hidden`}>
        <div
          className={`${height} rounded-full transition-all duration-1000 ease-out ${color}`}
          style={{
            width: `${Math.min(percentage, 100)}%`
          }}
        />
      </div>
    </div>
  );

  return (
    <div className="min-h-screen bg-gray-50 p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="flex flex-col md:flex-row items-start md:items-center justify-between mb-8 gap-4">
          <div className="flex items-center gap-4">
            <div className="p-2 bg-gradient-to-br from-blue-950 to-purple-800 rounded-xl flex items-center justify-center">
              <svg xmlns="http://www.w3.org/2000/svg" width="233" height="36" viewBox="0 0 233 36" fill="none"><g clip-path="url(#clip0_265_3161)"><path d="M65.4018 32.4481V15.1367H58.8529C58.2034 15.1367 57.8252 14.7585 57.8252 14.1089V10.3022C57.8252 9.65263 58.2034 9.27441 58.8529 9.27441H78.0842C78.7337 9.27441 79.1119 9.65263 79.1119 10.3022V14.1089C79.1119 14.7585 78.7337 15.1367 78.0842 15.1367H71.5354V32.4481C71.5354 33.0976 71.1572 33.4759 70.5076 33.4759H66.4295C65.78 33.4759 65.4018 33.0976 65.4018 32.4481Z" fill="white" /><path d="M100.604 27.2722L104.066 32.2424C104.547 32.9618 104.238 33.4757 103.416 33.4757H98.7872C98.2404 33.4757 97.8293 33.2701 97.5539 32.7891L94.5364 27.9546H87.9547V32.4438C87.9547 33.0933 87.5765 33.4715 86.927 33.4715H82.8818C82.2322 33.4715 81.854 33.0933 81.854 32.4438V10.3061C81.854 9.65653 82.2322 9.27832 82.8818 9.27832H95.3257C102.183 9.27832 105.13 12.5013 105.13 18.3636V19.0132C105.13 22.9556 103.827 25.9731 100.604 27.2722ZM87.9547 22.1992H94.8119C97.3812 22.1992 98.9928 21.858 98.9928 18.9433V18.3965C98.9928 15.4818 97.484 15.1077 94.8119 15.1077H87.9547V22.2033V22.1992Z" fill="white" /><path d="M108.214 25.1468V17.6401C108.214 12.7028 110.853 9.27832 116.337 9.27832H123.98C129.464 9.27832 132.103 12.7069 132.103 17.6401C132.103 18.2567 131.725 18.598 131.075 18.598H127.03C126.38 18.598 126.002 18.3924 126.002 18.0142V17.4304C126.002 15.3379 125.283 15.0995 123.98 15.0995H116.337C115.034 15.0995 114.315 15.3379 114.315 17.4304V25.3482C114.315 27.4037 115.137 27.6463 116.337 27.6463H123.98C125.18 27.6463 126.002 27.4078 126.002 25.3482V24.8343C126.002 24.1848 126.38 23.8066 127.03 23.8066H131.075C131.725 23.8066 132.103 24.045 132.103 24.4561V25.1427C132.103 30.043 129.464 33.4715 123.98 33.4715H116.337C110.853 33.4715 108.214 30.043 108.214 25.1427V25.1468Z" fill="white" /><path d="M49.4224 26.8777V17.258C49.4224 14.4666 47.0627 10.9106 44.2631 10.9106H35.3792L35.371 8.67009C35.371 8.43988 35.3546 8.20966 35.3258 7.98767C35.2025 5.88284 34.4337 5.17575 33.9157 4.13977C33.6526 3.63001 32.481 1.73073 30.1459 0.781089C29.4471 0.501542 28.6454 0.291881 27.9342 0.160329C27.9301 0.156218 27.926 0.156218 27.9178 0.156218C27.5108 0.053443 27.0874 0 26.6516 0H19.0997C16.3001 0 14.0267 2.27338 14.0267 5.07297V6.58171C14.0267 7.35046 13.3977 7.98356 12.6249 7.98356H5.07297C2.27338 7.98356 0 10.2528 0 13.0483V26.8201C0 29.1963 1.64029 31.1942 3.852 31.7451C4.41932 33.9239 6.40082 35.5272 8.75642 35.5231L44.3535 35.4779C47.149 35.4697 49.4224 33.1963 49.4183 30.4008L49.4101 27.0134C49.4183 26.964 49.4183 26.9229 49.4183 26.8777H49.4224ZM42.0678 26.8201C42.0678 27.593 41.4347 28.2179 40.666 28.2179H5.07297C4.35766 28.2179 3.76567 27.6835 3.68345 26.9887C3.67523 26.9353 3.67112 26.8777 3.67112 26.8201V13.0483C3.67112 12.2795 4.3001 11.6547 5.07297 11.6547H8.11511C8.31655 11.63 8.52209 11.6177 8.72764 11.6177H13.2703C15.7616 11.2929 17.6896 9.15519 17.6896 6.58171V5.07297C17.6896 4.3001 18.3227 3.67112 19.0915 3.67112H26.6434C27.4162 3.67112 28.0452 4.3001 28.0452 5.07297V10.7914C28.0452 13.5868 30.3186 15.8643 33.1141 15.8643H40.666C41.4347 15.8643 42.0678 16.4892 42.0678 17.2621V26.8201Z" fill="#36C188" /><path d="M15.8808 19.5686C15.8808 21.4473 14.3515 22.9807 12.4646 22.9807C10.5776 22.9807 9.04834 21.4514 9.04834 19.5686C9.04834 17.6857 10.5817 16.1523 12.4646 16.1523C14.3474 16.1523 15.8808 17.6816 15.8808 19.5686Z" fill="#36C188" /><path d="M28.4687 19.5686C28.4687 21.4473 26.9312 22.9807 25.0525 22.9807C23.1737 22.9807 21.6362 21.4514 21.6362 19.5686C21.6362 17.6857 23.1696 16.1523 25.0525 16.1523C26.9353 16.1523 28.4687 17.6816 28.4687 19.5686Z" fill="#36C188" /><path d="M154.113 14.4414H152.057V20.0159H150.405V14.4414H148.386V13.167H154.109V14.4414H154.113Z" fill="white" /><path d="M159.058 17.1094H156.464V18.7456H159.531V20.0159H154.816V13.167H159.544V14.4414H156.469V15.8844H159.063V17.1053L159.058 17.1094Z" fill="white" /><path d="M162.528 17.4673L161.85 18.2114V20.0161H160.197V13.1672H161.85V16.1847L162.425 15.3131L163.901 13.1631H165.94L163.626 16.1929L165.94 20.012H163.984L162.524 17.4632L162.528 17.4673Z" fill="white" /><path d="M166.359 20.0159V13.167H168.818C169.697 13.167 170.368 13.3273 170.828 13.6521C171.288 13.9769 171.519 14.4455 171.519 15.0581C171.519 15.4116 171.436 15.7158 171.276 15.9666C171.112 16.2174 170.873 16.4023 170.557 16.5216C170.914 16.6161 171.19 16.7929 171.379 17.0478C171.572 17.3068 171.667 17.6192 171.667 17.9892C171.667 18.6593 171.453 19.1649 171.029 19.502C170.606 19.8391 169.977 20.0118 169.143 20.0159H166.363H166.359ZM168.012 16.0241H168.871C169.233 16.0241 169.488 15.9542 169.644 15.8268C169.796 15.6994 169.874 15.5062 169.874 15.2554C169.874 14.9676 169.792 14.7579 169.623 14.6305C169.459 14.5031 169.188 14.4414 168.818 14.4414H168.008V16.0282L168.012 16.0241ZM168.012 17.1053V18.7456H169.093C169.389 18.7456 169.619 18.6757 169.78 18.5401C169.94 18.4044 170.018 18.2112 170.018 17.9645C170.018 17.3931 169.734 17.1053 169.167 17.1053H168.012Z" fill="white" /><path d="M178.372 16.7315C178.372 17.3974 178.248 17.9894 178.002 18.5033C177.755 19.0172 177.402 19.4118 176.945 19.6914C176.489 19.9668 175.971 20.1066 175.387 20.1066C174.803 20.1066 174.285 19.9709 173.833 19.7037C173.381 19.4365 173.032 19.0542 172.777 18.5526C172.526 18.0552 172.39 17.4838 172.378 16.8342V16.4478C172.378 15.7777 172.501 15.1857 172.748 14.6719C172.995 14.158 173.348 13.7633 173.804 13.4838C174.265 13.2042 174.787 13.0645 175.375 13.0645C175.963 13.0645 176.472 13.2042 176.929 13.4797C177.385 13.7551 177.739 14.1498 177.989 14.6595C178.24 15.1693 178.368 15.753 178.372 16.4108V16.7274V16.7315ZM176.694 16.4437C176.694 15.7613 176.579 15.2474 176.353 14.8938C176.127 14.5403 175.802 14.3635 175.379 14.3635C174.553 14.3635 174.113 14.9843 174.068 16.2258V16.7274C174.063 17.3974 174.174 17.9154 174.396 18.2731C174.618 18.6307 174.951 18.8157 175.387 18.8157C175.823 18.8157 176.127 18.639 176.353 18.2854C176.579 17.9319 176.694 17.4221 176.699 16.7561V16.4396L176.694 16.4437Z" fill="white" /><path d="M184.197 14.4414H182.142V20.0159H180.489V14.4414H178.471V13.167H184.193V14.4414H184.197Z" fill="white" /><path d="M189.553 17.5945H188.661V20.0159H187.009V13.167H189.706C190.52 13.167 191.153 13.3479 191.609 13.7055C192.065 14.0673 192.296 14.573 192.296 15.2307C192.296 15.7076 192.201 16.1022 192.008 16.4147C191.815 16.7271 191.514 16.982 191.103 17.1752L192.534 19.9419V20.0118H190.766L189.558 17.5904L189.553 17.5945ZM188.661 16.3242H189.706C190.018 16.3242 190.256 16.242 190.413 16.0776C190.569 15.9131 190.651 15.6829 190.651 15.3869C190.651 15.0909 190.573 14.8607 190.413 14.6922C190.252 14.5236 190.018 14.4414 189.706 14.4414H188.661V16.3283V16.3242Z" fill="white" /><path d="M199.091 16.7315C199.091 17.3974 198.968 17.9894 198.722 18.5033C198.475 19.0172 198.121 19.4118 197.665 19.6914C197.209 19.9668 196.691 20.1066 196.107 20.1066C195.523 20.1066 195.005 19.9709 194.553 19.7037C194.101 19.4365 193.751 19.0542 193.496 18.5526C193.246 18.0552 193.11 17.4838 193.098 16.8342V16.4478C193.098 15.7777 193.221 15.1857 193.468 14.6719C193.714 14.158 194.068 13.7633 194.524 13.4838C194.985 13.2042 195.507 13.0645 196.095 13.0645C196.682 13.0645 197.192 13.2042 197.649 13.4797C198.105 13.7551 198.458 14.1498 198.709 14.6595C198.96 15.1693 199.087 15.753 199.091 16.4108V16.7274V16.7315ZM197.414 16.4437C197.414 15.7613 197.299 15.2474 197.073 14.8938C196.847 14.5403 196.522 14.3635 196.099 14.3635C195.272 14.3635 194.833 14.9843 194.787 16.2258V16.7274C194.783 17.3974 194.894 17.9154 195.116 18.2731C195.338 18.6307 195.671 18.8157 196.107 18.8157C196.543 18.8157 196.847 18.639 197.073 18.2854C197.299 17.9319 197.414 17.4221 197.418 16.7561V16.4396L197.414 16.4437Z" fill="white" /><path d="M199.946 20.0159V13.167H202.405C203.284 13.167 203.955 13.3273 204.415 13.6521C204.875 13.9769 205.106 14.4455 205.106 15.0581C205.106 15.4116 205.023 15.7158 204.863 15.9666C204.699 16.2174 204.46 16.4023 204.144 16.5216C204.501 16.6161 204.777 16.7929 204.966 17.0478C205.159 17.3068 205.254 17.6192 205.254 17.9892C205.254 18.6593 205.04 19.1649 204.616 19.502C204.193 19.8391 203.564 20.0118 203.729 20.0159H199.95H199.946ZM201.595 16.0241H202.454C202.816 16.0241 203.071 15.9542 203.227 15.8268C203.379 15.6994 203.457 15.5062 203.457 15.2554C203.457 14.9676 203.375 14.7579 203.206 14.6305C203.042 14.5031 202.771 14.4414 202.401 14.4414H201.591V16.0282L201.595 16.0241ZM201.595 17.1053V18.7456H202.676C202.972 18.7456 203.202 18.6757 203.363 18.5401C203.523 18.4044 203.601 18.2112 203.601 17.9645C203.601 17.3931 203.317 17.1053 202.75 17.1053H201.595Z" fill="white" /><path d="M211.955 16.7315C211.955 17.3974 211.831 17.9894 211.585 18.5033C211.338 19.0172 210.985 19.4118 210.528 19.6914C210.072 19.9668 209.554 20.1066 208.97 20.1066C208.386 20.1066 207.868 19.9709 207.416 19.7037C206.964 19.4365 206.615 19.0542 206.36 18.5526C206.109 18.0552 205.973 17.4838 205.961 16.8342V16.4478C205.961 15.7777 206.084 15.1857 206.331 14.6719C206.578 14.158 206.931 13.7633 207.387 13.4838C207.848 13.2042 208.37 13.0645 208.958 13.0645C209.546 13.0645 210.055 13.2042 210.512 13.4797C210.968 13.7551 211.322 14.1498 211.572 14.6595C211.823 15.1693 211.951 15.753 211.955 16.4108V16.7274V16.7315ZM210.277 16.4437C210.277 15.7613 210.162 15.2474 209.936 14.8938C209.71 14.5403 209.385 14.3635 208.962 14.3635C208.136 14.3635 207.696 14.9843 207.651 16.2258V16.7274C207.646 17.3974 207.757 17.9154 207.979 18.2731C208.201 18.6307 208.534 18.8157 208.97 18.8157C209.406 18.8157 209.71 18.639 209.936 18.2854C210.162 17.9319 210.277 17.4221 210.282 16.7561V16.4396L210.277 16.4437Z" fill="white" /><path d="M217.784 14.4414H215.728V20.0159H214.076V14.4414H212.057V13.167H217.78V14.4414H217.784Z" fill="white" /><path d="M220.238 20.0159H218.594V13.167H220.238V20.0159Z" fill="white" /><path d="M226.927 17.6971C226.906 18.174 226.775 18.5933 226.54 18.9592C226.306 19.3251 225.977 19.6088 225.55 19.8102C225.126 20.0116 224.641 20.1103 224.094 20.1103C223.194 20.1103 222.487 19.8184 221.969 19.2305C221.451 18.6427 221.192 17.8164 221.192 16.7475V16.4104C221.192 15.7403 221.307 15.1524 221.541 14.6509C221.776 14.1493 222.113 13.7588 222.549 13.4834C222.988 13.2079 223.494 13.0723 224.074 13.0723C224.904 13.0723 225.574 13.2901 226.076 13.73C226.581 14.1658 226.869 14.7701 226.935 15.5389H225.291C225.278 15.1237 225.171 14.8236 224.974 14.6386C224.777 14.4577 224.477 14.3672 224.07 14.3672C223.663 14.3672 223.358 14.5193 223.165 14.8277C222.972 15.136 222.873 15.6252 222.865 16.2994V16.7845C222.865 17.5163 222.956 18.0384 223.141 18.3508C223.325 18.6632 223.638 18.8194 224.09 18.8194C224.468 18.8194 224.76 18.729 224.962 18.5522C225.163 18.3755 225.27 18.0918 225.282 17.7012H226.923L226.927 17.6971Z" fill="white" /><path d="M231.239 18.1946C231.239 17.952 231.153 17.767 230.984 17.6314C230.812 17.4957 230.512 17.3559 230.084 17.212C229.657 17.0682 229.303 16.9284 229.032 16.7927C228.144 16.357 227.7 15.7567 227.7 14.9962C227.7 14.618 227.811 14.2809 228.033 13.9931C228.255 13.7012 228.567 13.4793 228.97 13.3148C229.373 13.1545 229.829 13.0723 230.335 13.0723C230.84 13.0723 231.268 13.1586 231.659 13.3354C232.049 13.5121 232.353 13.7629 232.567 14.0836C232.785 14.4083 232.892 14.7783 232.892 15.1977H231.247C231.247 14.9181 231.161 14.7043 230.993 14.5481C230.82 14.396 230.59 14.3179 230.298 14.3179C230.006 14.3179 229.772 14.3837 229.599 14.5111C229.426 14.6427 229.344 14.8071 229.344 15.0085C229.344 15.1853 229.439 15.3415 229.628 15.4854C229.817 15.6293 230.146 15.7773 230.618 15.9294C231.091 16.0815 231.482 16.246 231.786 16.4227C232.526 16.8503 232.896 17.4381 232.896 18.1863C232.896 18.7866 232.67 19.2552 232.218 19.5964C231.765 19.9376 231.149 20.1103 230.359 20.1103C229.805 20.1103 229.303 20.0116 228.851 19.8102C228.399 19.6088 228.061 19.3374 227.835 18.9921C227.609 18.6468 227.494 18.248 227.494 17.7958H229.151C229.151 18.1617 229.245 18.433 229.435 18.6057C229.624 18.7783 229.932 18.8647 230.359 18.8647C230.631 18.8647 230.849 18.8071 231.005 18.6879C231.161 18.5687 231.243 18.4042 231.243 18.1905L231.239 18.1946Z" fill="white" /><path d="M153.936 29.4386C153.858 30.158 153.599 30.713 153.163 31.0953C152.727 31.4776 152.144 31.6708 151.416 31.6708C150.906 31.6708 150.458 31.5434 150.068 31.2885C149.677 31.0336 149.377 30.676 149.163 30.2073C148.949 29.7428 148.842 29.2084 148.838 28.6123V27.7243C148.838 27.1159 148.945 26.5773 149.159 26.1046C149.373 25.6359 149.677 25.27 150.08 25.0151C150.479 24.7603 150.939 24.6328 151.461 24.6328C152.193 24.6328 152.773 24.8301 153.2 25.2248C153.624 25.6195 153.87 26.1662 153.94 26.8651H153.356C153.212 25.7017 152.579 25.122 151.461 25.122C150.84 25.122 150.347 25.3522 149.977 25.8168C149.607 26.2813 149.426 26.9226 149.426 27.7407V28.5794C149.426 29.3687 149.607 30.0018 149.965 30.4704C150.322 30.9391 150.808 31.1775 151.42 31.1775C152.033 31.1775 152.481 31.0336 152.789 30.7418C153.097 30.4499 153.286 30.0182 153.356 29.4386H153.94H153.936Z" fill="white" /><path d="M160.415 31.5726H159.831V28.2879H155.918V31.5726H155.338V24.7236H155.918V27.7987H159.831V24.7236H160.415V31.5726Z" fill="white" /><path d="M165.936 29.6486H162.828L162.121 31.5726H161.513L164.103 24.7236H164.653L167.243 31.5726H166.643L165.932 29.6486H165.936ZM163.009 29.1594H165.755L164.382 25.4307L163.009 29.1594Z" fill="white" /><path d="M168.838 31.0833H172.222V31.5726H168.259V24.7236H168.843V31.0833H168.838Z" fill="white" /><path d="M173.916 31.0833H177.299V31.5726H173.336V24.7236H173.92V31.0833H173.916Z" fill="white" /><path d="M182.187 28.2918H178.984V31.0873H182.668V31.5765H178.409V24.7275H182.647V25.2167H178.988V27.8026H182.191V28.2918H182.187Z" fill="white" /><path d="M188.982 31.5726H188.402L184.464 25.7144V31.5726H183.88V24.7236H184.464L188.407 30.59V24.7236H188.982V31.5726Z" fill="white" /><path d="M195.663 30.7584C195.461 31.0421 195.149 31.2682 194.721 31.4244C194.294 31.5847 193.813 31.6628 193.286 31.6628C192.76 31.6628 192.275 31.5354 191.856 31.2764C191.436 31.0174 191.116 30.6556 190.886 30.187C190.655 29.7183 190.54 29.1757 190.536 28.5672V27.7039C190.536 26.7543 190.775 26.002 191.256 25.4552C191.737 24.9043 192.378 24.6289 193.188 24.6289C193.895 24.6289 194.462 24.8098 194.89 25.1674C195.317 25.5251 195.576 26.0225 195.654 26.6515H195.075C194.988 26.15 194.787 25.7677 194.462 25.5087C194.137 25.2497 193.714 25.1181 193.192 25.1181C192.551 25.1181 192.045 25.3442 191.671 25.8005C191.297 26.2569 191.112 26.8982 191.112 27.7245V28.5344C191.112 29.0565 191.202 29.5169 191.379 29.9156C191.556 30.3144 191.811 30.6269 192.139 30.8447C192.468 31.0626 192.851 31.1736 193.278 31.1736C193.776 31.1736 194.203 31.0955 194.561 30.9393C194.791 30.8365 194.964 30.7132 195.079 30.5775V28.8098H193.241V28.3206H195.658V30.7584H195.663Z" fill="white" /><path d="M201.089 28.2918H197.887V31.0873H201.57V31.5765H197.311V24.7275H201.549V25.2167H197.891V27.8026H201.093V28.2918H201.089Z" fill="white" /><path d="M142.541 9.96094H141.702V36H142.541V9.96094Z" fill="white" /></g><defs><clipPath id="clip0_265_3161"><rect width="232.888" height="36" fill="white" /></clipPath></defs></svg>
            </div>
            <div>
              <h1 className="text-2xl font-bold text-gray-800">
                Système Intelligent de Tri des Déchets
              </h1>
              <p className="text-sm text-gray-500">
                Surveillance en temps réel • Groupe Innovators
              </p>
            </div>
          </div>

          <div className="flex items-center gap-3">
            <button
              onClick={resetCounters}
              className="flex items-center gap-2 px-4 py-2 bg-white rounded-lg border border-gray-200 hover:bg-gray-50 transition-colors text-sm"
            >
              <RefreshCw className="w-4 h-4" />
              Réinitialiser
            </button>
            <div className="flex items-center gap-2 px-4 py-2 bg-green-50 rounded-lg border border-green-100">
              <div className="w-2 h-2 bg-green-500 rounded-full"></div>
              <span className="text-sm font-medium text-green-700">Système Actif</span>
            </div>
          </div>
        </div>

        {/* Grille principale */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
          {/* Colonne de gauche - Statistiques clés */}
          <div className="space-y-6">
            {/* Carte de performance globale */}
            <div className="bg-white rounded-xl p-6 shadow-sm border border-gray-100">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-semibold text-gray-800 flex items-center gap-2">
                  <Activity className="w-5 h-5 text-blue-600" />
                  Performance Globale
                </h3>
                <span className="text-xs px-2 py-1 bg-blue-50 text-blue-600 rounded-full">Temps réel</span>
              </div>

              <div className="space-y-4">
                <div>
                  <div className="text-3xl font-bold text-gray-800 mb-1">
                    {totalProcessed.toLocaleString()}
                  </div>
                  <p className="text-sm text-gray-500">Déchets traités</p>
                </div>

                <ProgressBar
                  percentage={totalWaste > 0 ? ((wasteCounts.recyclable + wasteCounts.organic) / totalWaste) * 100 : 0}
                  color="bg-gradient-to-r from-emerald-500 to-green-400"
                  height="h-3"
                  showLabel={true}
                  label="Taux de recyclage"
                />

                <div className="grid grid-cols-2 gap-4 pt-4 border-t border-gray-100">
                  <div>
                    <div className="text-xl font-bold text-emerald-600">
                      {wasteCounts.recyclable.toLocaleString()}
                    </div>
                    <p className="text-xs text-gray-500">Recyclables</p>
                  </div>
                  <div>
                    <div className="text-xl font-bold text-amber-500">
                      {wasteCounts.organic.toLocaleString()}
                    </div>
                    <p className="text-xs text-gray-500">Organiques</p>
                  </div>
                </div>
              </div>
            </div>

            {/* Carte efficacité */}
            <div className="bg-white rounded-xl p-6 shadow-sm border border-gray-100">
              <h3 className="font-semibold text-gray-800 mb-4 flex items-center gap-2">
                <Gauge className="w-5 h-5 text-purple-600" />
                Efficacité du Système
              </h3>

              <div className="flex items-center justify-between mb-2">
                <span className="text-sm font-medium text-gray-600">Précision de tri</span>
                <span className="text-sm font-bold text-gray-800">{efficiency.toFixed(1)}%</span>
              </div>
              <ProgressBar
                percentage={efficiency}
                color="bg-gradient-to-r from-purple-500 to-indigo-500"
                height="h-2"
              />

              <div className="mt-6 grid grid-cols-3 gap-4">
                <div className="text-center">
                  <div className="text-xs text-gray-500 mb-1">Électronique</div>
                  <div className="text-lg font-bold text-blue-600">
                    {totalWaste > 0 ? ((wasteCounts.electronic / totalWaste) * 100).toFixed(1) : 0}%
                  </div>
                </div>
                <div className="text-center">
                  <div className="text-xs text-gray-500 mb-1">Résiduels</div>
                  <div className="text-lg font-bold text-red-500">
                    {totalWaste > 0 ? ((wasteCounts.residual / totalWaste) * 100).toFixed(1) : 0}%
                  </div>
                </div>
                <div className="text-center">
                  <div className="text-xs text-gray-500 mb-1">Erreurs</div>
                  <div className="text-lg font-bold text-gray-600">
                    {totalWaste > 0 ? (100 - efficiency).toFixed(1) : 0}%
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Colonne centrale - Visualisation des catégories */}
          <div className="lg:col-span-2">
            <div className="bg-white rounded-xl p-6 shadow-sm border border-gray-100 h-full">
              <h3 className="font-semibold text-gray-800 mb-4 flex items-center gap-2">
                <Database className="w-5 h-5 text-blue-600" />
                Répartition par Catégories
              </h3>

              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                {wasteCategories.map((category) => {
                  const percentage = totalWaste > 0 ? (category.count / totalWaste) * 100 : 0;
                  return (
                    <div
                      key={category.type}
                      id={`card-${category.type}`}
                      className={`
                        ${clickedCard === category.type ? category.darkBgColor : category.bgColor}
                        rounded-xl p-5 border border-gray-100 
                        hover:shadow-md transition-all duration-300 
                        transform hover:-translate-y-1
                        ${clickedCard === category.type ? 'scale-[1.02]' : ''}
                        relative overflow-hidden
                      `}
                    >
                      {/* Animation +1 */}
                      {floatingPlusOnes
                        .filter(po => po.type === category.type)
                        .map(po => (
                          <div
                            key={po.id}
                            className={`absolute text-2xl font-bold ${category.color}`}
                            style={{
                              top: `${po.top}px`,
                              left: `${po.left}px`,
                              opacity: po.opacity,
                              transform: `translateY(-${(1 - po.opacity) * 50}px)`,
                              transition: 'opacity 0.05s, transform 0.05s',
                              pointerEvents: 'none'
                            }}
                          >
                            +1
                          </div>
                        ))}

                      <div className="flex items-start justify-between mb-3">
                        <div>
                          <div className={`${category.color} p-2 rounded-lg inline-flex mb-2`}>
                            {category.icon}
                          </div>
                          <h3 className="font-semibold text-gray-800">
                            {category.title}
                          </h3>
                          <p className="text-xs text-gray-500 mt-1">
                            {category.description}
                          </p>
                        </div>
                        <div className="text-right">
                          <p className={`text-2xl font-bold ${category.color}`}>
                            {category.count}
                          </p>
                          <p className="text-xs text-gray-500">
                            {percentage.toFixed(1)}% du total
                          </p>
                        </div>
                      </div>

                      <ProgressBar
                        percentage={percentage}
                        color={category.type === 'recyclable' ? 'bg-emerald-500' :
                          category.type === 'organic' ? 'bg-amber-500' :
                            category.type === 'electronic' ? 'bg-blue-500' : 'bg-red-500'}
                        height="h-2"
                        animated={clickedCard === category.type}
                      />
                    </div>
                  );
                })}
              </div>
              <br />
              <div className="text-2xl text-blue-500 font-bold italic text-center mt-2">
                © {new Date().getFullYear()} Team Innovators IT
              </div>
            </div>

            {/* Graphiques de performance */}

          </div>
        </div>
      </div>
    </div>
  );
};

export default ConvoyeurDashboard;