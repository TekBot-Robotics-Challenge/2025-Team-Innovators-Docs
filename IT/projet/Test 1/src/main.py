from Robots import *

if __name__ == "__main__":
    robot_menager = RobotDomestique("Innovator", "noir", ["nettoyage", "rangement"])
    robot_menager.move(5, 10)
    robot_menager.travailler()