// ========================================
// VERSION ROSMASTER X3 CORRIGÉE - FINALE TRC 2025
// ========================================
import React, { useState } from 'react';
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
  MapPin,
  Eye,
  QrCode,
  Swords,
  Trophy
} from 'lucide-react';

// ========================================
// TABLE DES MATIÈRES - ROSMASTER X3 SEULEMENT
// ========================================
export const TOC_ROSMaster = () => {
  const [isOpen, setIsOpen] = useState(true);

  return (
    <div className="bg-gray-50 border border-gray-200 rounded-lg p-4 mb-6">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-semibold text-gray-800 hover:text-blue-600"
      >
        <div className="flex items-center gap-2">
          <BookOpen size={20} />
          Table des matières - ROSMaster X3 TRC 2025
        </div>
        <ChevronDown size={16} className={`transform transition-transform ${isOpen ? "rotate-180" : ""}`} />
      </button>
      {isOpen && (
        <div className="mt-4 space-y-1 text-sm">
          {[
            ["intro-rosmaster", "1. Introduction ROSMaster X3"],
            ["specs-rosmaster", "2. Spécifications Techniques"],
            ["code-rosmaster", "3. Code Navigation & QR"],
            ["simulation-rosmaster", "4. Simulation Gazebo TRC"],
            ["performance-rosmaster", "5. Performance Finale"]
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
// PAGE PRINCIPALE - ROSMASTER X3 UNIQUEMENT
// ========================================
const Documentation_ROSMaster = () => {
  return (
    <div className="max-w-5xl mx-auto p-6">
      {/* En-tête unique */}
      <div className="bg-gradient-to-r from-purple-600 to-blue-600 text-white p-4 rounded-lg mb-6">
        <div className="flex items-center gap-3">
          <Cpu size={32} className="text-yellow-300" />
          <div>
            <h1 className="text-3xl font-bold">ROSMaster X3 - TRC 2025 Finale</h1>
            <p className="text-purple-100">Robot mobile - Documentation technique</p>
          </div>
        </div>
      </div>

      <TOC_ROSMaster />

      {/* Section 1 : Introduction */}
      <section id="intro-rosmaster" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <Zap size={20} />
          1. Introduction ROSMaster X3
        </h2>
        <div className="bg-purple-50 border-l-4 border-purple-400 p-4">
          <p className="text-purple-800">
            Le ROSMaster X3 a été notre robot mobile pour la collecte des déchets dans la ville EcoCity.
          </p>
          <ul className="mt-3 text-purple-700 space-y-1">
            <li>• Navigation autonome sur arène 5m×5m</li>
            <li>• Scan de 10 QR codes (quartiers)</li>
            <li>• Collecte de 20 cubes en 5 minutes</li>
          </ul>
        </div>
      </section>

      {/* Section 2 : Spécifications */}
      <section id="specs-rosmaster" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <Wrench size={20} />
          2. Spécifications Techniques ROSMaster X3
        </h2>
        <div className="bg-purple-50 border-2 border-purple-400 rounded-lg p-6">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            <div className="bg-white p-4 rounded border">
              <h3 className="font-bold mb-2 flex items-center gap-2"><Package size={16} /> Hardware</h3>
              <ul className="text-sm space-y-1">
                <li>Processeur: Raspberry Pi 4B (4GB)</li>
                <li>Caméra: Intel RealSense D435i</li>
                <li>LIDAR: YDLIDAR X4 (10m range)</li>
                <li>Moteurs: 4x moteurs Brushless 25W</li>
                <li>Batterie: LiPo 12V 5000mAh</li>
              </ul>
            </div>
            <div className="bg-white p-4 rounded border">
              <h3 className="font-bold mb-2 flex items-center gap-2"><GitBranch size={16} /> Software</h3>
              <ul className="text-sm space-y-1">
                <li>ROS Noetic / Ubuntu 18.04</li>
                <li>OpenCV 4.5.4 (CUDA enabled)</li>
                <li>Navigation Stack ROS</li>
                <li>gmapping / amcl</li>
                <li>Python 3.8</li>
              </ul>
            </div>
          </div>
        </div>
      </section>

      {/* Section 3 : Code Navigation */}
      <section id="code-rosmaster" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <Code size={20} />
          3. Code Navigation & Scan QR
        </h2>
        <CodeViewer
          code={`#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from pyzbar.pyzbar import decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ROSMasterX3Navigator:
    def __init__(self):
        rospy.init_node('rosmaster_navigator')
        self.bridge = CvBridge()
        
        # Publishers & Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.qr_callback)
        
        # État
        self.quartiers_scannes = {}
        self.position_actuelle = (0, 0)
        
    def qr_callback(self, img_msg):
        """Scan QR codes des 10 quartiers EcoCity"""
        frame = self.bridge.imgmsg_to_cv2(img_msg)
        decoded = decode(frame)
        
        for q in decoded:
            data = q.data.decode('utf-8')
            # Format: "quartier:Haie-Vive|type:menager|dechets:5"
            self.quartiers_scannes[data] = True
            rospy.loginfo(f"QR détecté: {data}")
            
    def scan_callback(self, scan_msg):
        """Évitement d'obstacles avec LIDAR"""
        ranges = scan_msg.ranges
        front_distance = min(ranges[0:30] + ranges[-30:])
        
        if front_distance < 0.5:  # Obstacle proche
            self.stop_robot()
        else:
            self.move_forward()
            
    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.3  # Vitesse 30cm/s
        self.cmd_pub.publish(cmd)
        
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    navigator = ROSMasterX3Navigator()
    rospy.spin()`}
          language="python"
        />
      </section>

      {/* Section 4 : Simulation */}
      <section id="simulation-rosmaster" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <Eye size={20} />
          4. Simulation Gazebo TRC 2025
        </h2>
        <div className="bg-blue-50 border-2 border-blue-400 rounded-lg p-6">
          <div className="bg-white p-4 rounded border mb-4">
            <h3 className="font-bold mb-2 flex items-center gap-2"><Package size={16} /> Dépot fourni par TRC</h3>
            <ul className="text-sm space-y-1">
              <li>Repository: <code>github.com/tekbot-robotics/trc2025_simulation</code></li>
              <li>Launch file: <code>trc_arena.launch.py</code></li>
              <li>World: <code>ecocity_5x5.world</code></li>
              <li>Robot URDF: <code>rosmaster_x3.urdf.xacro</code></li>
            </ul>
          </div>
          <CodeViewer
            code={`# Lancer la simulation officielle TRC
roslaunch trc2025_simulation trc_arena.launch

# Déplacer le robot en téléopération
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel

# Visualiser la carte des QR codes
rosrun rviz rviz -d $(rospack find trc2025_simulation)/rviz/qr_mapping.rviz`}
            language="bash"
          />
        </div>
      </section>

      {/* Section 5 : Performance */}
      <section id="performance-rosmaster" className="mb-12">
        <h2 className="text-2xl font-bold text-gray-800 mb-4 flex items-center gap-2">
          <BarChart size={20} />
          5. Performance Finale
        </h2>
        <div className="bg-green-50 border-2 border-green-400 rounded-lg p-6">
          <div className="grid grid-cols-1 md:grid-cols-4 gap-4 text-center">
            <div><div className="text-2xl font-bold text-green-600">17/20</div><div className="text-xs">Cubes collectés</div></div>
            <div><div className="text-2xl font-bold text-blue-600">8/10</div><div className="text-xs">QR codes scannés</div></div>
            <div><div className="text-2xl font-bold text-purple-600">4m20s</div><div className="text-xs">Temps collecte</div></div>
            <div><div className="text-2xl font-bold text-red-600">0</div><div className="text-xs">Collisions</div></div>
          </div>
        </div>
      </section>
    </div>
  );
};

export default Documentation_ROSMaster;