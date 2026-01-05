// ========================================
// VERSION DOFBOT CORRIGÉE - TRC 2025 Finale
// ========================================
import { useState } from "react";
import { CodeViewer } from "../../components/CodeViewer";
import { 
  BookOpen, 
  ChevronDown, 
  Cpu, 
  Camera, 
  GitBranch, 
  Zap, 
  Package,
  Wrench,
  Code,
  Info,
  ImageIcon,
  AlertTriangle,
  BarChart,
  Activity,
  Move,
  Eye,
  Brain, // ⚠️ MANQUANT DANS VOTRE VERSION
  Settings // ⚠️ MANQUANT DANS VOTRE VERSION
} from 'lucide-react';

// ========================================
// TABLE DES MATIÈRES
// ========================================
export const TOC_Dofbot = () => {
  const [isOpen, setIsOpen] = useState(true);

  return (
    <div className="bg-gray-50 border border-gray-200 rounded-lg p-4 mb-6">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-semibold text-gray-800 hover:text-blue-600"
      >
        <div className="flex items-center gap-2">
          <BookOpen size={20} />
          Table des matières - Dofbot TRC 2025
        </div>
        <ChevronDown size={16} className={`transform transition-transform ${isOpen ? "rotate-180" : ""}`} />
      </button>
      {isOpen && (
        <div className="mt-4 space-y-1 text-sm">
          {[
            ["intro-dofbot", "1. Introduction Dofbot"],
            ["specs-dofbot", "2. Spécifications Techniques"],
            ["code-dofbot", "3. Code ROS Classification"],
            ["performance-dofbot", "4. Performance & Résultats"]
          ].map(([href, label]) => (
            <a key={href} href={`#${href}`} className="block text-blue-600 hover:underline">
              {label}
            </a>
          ))}
        </div>
      )}
    </div>
  );
};

// ========================================
// CODE EXEMPLE
// ========================================
type CodeExampleDofbotProps = {
  title: string;
  code: string;
  description: string;
};

export const CodeExample_Dofbot = ({ title, code, description }: CodeExampleDofbotProps) => (
  <div className="bg-white border border-gray-200 rounded-lg my-4">
    <div className="bg-gray-50 px-4 py-2 border-b border-gray-200 font-semibold text-gray-800 flex items-center gap-2">
      <Code size={16} />
      {title}
    </div>
    <div className="p-4">
      <div className="text-sm text-gray-600 mb-3 flex items-start gap-2">
        <Info size={14} className="mt-0.5" />
        {description}
      </div>
      <pre className="bg-gray-900 text-green-400 p-3 rounded text-sm overflow-x-auto">
        <code>{code}</code>
      </pre>
    </div>
  </div>
);

// ========================================
// PAGE PRINCIPALE - STRUCTURE CORRIGÉE
// ========================================
const Documentation_Dofbot = () => {
  return (
    <div className="max-w-5xl mx-auto p-6">
      {/* En-tête */}
      <div className="bg-gradient-to-r from-orange-600 to-red-600 text-white p-4 rounded-lg mb-6">
        <div className="flex items-center gap-3">
          <Cpu size={32} className="text-yellow-300" />
          <div>
            <h1 className="text-3xl font-bold">Dofbot Jetson Nano - TRC 2025</h1>
            <p className="text-orange-100">Documentation technique - Station de tri finale</p>
          </div>
        </div>
      </div>

      <TOC_Dofbot />

      {/* Section 1 : Introduction */}
      <section id="intro-dofbot" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <Zap size={20} />
          1. Introduction Dofbot
        </h2>
        <div className="bg-orange-50 border-l-4 border-orange-400 p-4">
          <p className="text-orange-800">
            Le bras robotique Dofbot Jetson Nano a été le composant central de notre station de tri lors de la finale TRC 2025.
          </p>
        </div>
      </section>

      {/* Section 2 : Spécifications */}
      <section id="specs-dofbot" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <Wrench size={20} />
          2. Spécifications Techniques Dofbot
        </h2>
        <div className="bg-orange-50 border-2 border-orange-400 rounded-lg p-6">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            <div className="bg-white p-4 rounded border">
              <h3 className="font-bold mb-2 flex items-center gap-2"><Package size={16} /> Hardware</h3>
              <ul className="text-sm space-y-1">
                <li>Jetson Nano: 4GB RAM, 128 core GPU</li>
                <li>Caméra IMX219 8MP</li>
                <li>6 servos MG996R</li>
              </ul>
            </div>
            <div className="bg-white p-4 rounded border">
              <h3 className="font-bold mb-2 flex items-center gap-2"><GitBranch size={16} /> Software</h3>
              <ul className="text-sm space-y-1">
                <li>ROS Noetic / Python 3.8</li>
                <li>PyTorch 1.8.0 + TensorRT</li>
                <li>YOLOv5s custom</li>
              </ul>
            </div>
          </div>
        </div>
      </section>

      {/* Section 3 : Code ROS */}
      <section id="code-dofbot" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <Code size={20} />
          3. Code ROS Classification
        </h2>
        <CodeExample_Dofbot
          title="dofbot_classifier.py"
          description="Node ROS complet avec YOLOv5 + TensorRT"
          code={`#!/usr/bin/env python3
import rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class DofbotClassifier:
    def __init__(self):
        rospy.init_node('dofbot_classifier')
        self.model = torch.load('/home/tekbot/dofbot_yolov5s_trt.pt')
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/image', Image, self.callback)
        
    def callback(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg)
        results = self.model(frame)
        # Classification ici

if __name__ == '__main__':
    rospy.spin()`}
        />
      </section>

      {/* Section 4 : Performance */}
      <section id="performance-dofbot" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <BarChart size={20} />
          4. Performance & Résultats
        </h2>
        <div className="bg-green-50 border-2 border-green-400 rounded-lg p-6">
          <div className="grid grid-cols-4 gap-4 text-center">
            <div><div className="text-2xl font-bold text-green-600">18/20</div><div className="text-xs">Cubes triés</div></div>
            <div><div className="text-2xl font-bold text-blue-600">94.3%</div><div className="text-xs">Précision</div></div>
            <div><div className="text-2xl font-bold text-purple-600">2.1s</div><div className="text-xs">Temps moyen</div></div>
            <div><div className="text-2xl font-bold text-red-600">1</div><div className="text-xs">Erreurs</div></div>
          </div>
        </div>
      </section>
    </div>
  );
};

export default Documentation_Dofbot;