import random

class Robots:
    """
    Classe de base représentant un robot générique.
    Gère les déplacements, l'énergie et l'affichage d'état.
    """

    def __init__(self, nom, couleur):
        """
        Initialise un robot avec un nom, une couleur et une énergie initiale.
        """
        self.nom = nom
        self.couleur = couleur
        self.__energie = 150
        self.__x = 0
        self.__y = 0

    @property
    def energie(self):
        """
        Retourne le niveau d'énergie du robot.
        """
        return self.__energie

    @energie.setter
    def energie(self, valeur):
        """
        Définit le niveau d'énergie du robot si la valeur est valide (0–200).
        """
        if 0 <= valeur <= 200:
            self.__energie = valeur
        else:
            print(f"Erreur: Énergie doit être entre 0 et 200 (valeur: {valeur})")

    @property
    def x(self):
        """Coordonnée X actuelle du robot."""
        return self.__x

    @property
    def y(self):
        """Coordonnée Y actuelle du robot."""
        return self.__y

    def move(self, x, y):
        """
        Déplace le robot à une nouvelle position (x, y) en consommant 10 d'énergie.
        """
        if self.__energie >= 10:
            self.__x = x
            self.__y = y
            self.__energie -= 10
            print(f"{self.nom} s'est déplacé à la position ({x}, {y})")
        else:
            print(f"{self.nom} n'a pas assez d'énergie pour se déplacer")

    def recharger(self):
        """
        Recharge l'énergie du robot à 100%.
        """
        self.__energie = 100
        print(f"{self.nom} a été rechargé à 100% d'énergie")

    def show(self):
        """
        Affiche l'état actuel du robot : nom, couleur, énergie et position.
        """
        print(f"Robot: {self.nom}")
        print(f"Couleur: {self.couleur}")
        print(f"Énergie: {self.__energie}%")
        print(f"Position: ({self.__x}, {self.__y})")

    def action_speciale(self):
        """
        Méthode polymorphe : action spéciale par défaut du robot.
        À redéfinir dans les sous-classes.
        """
        print(f"{self.nom} effectue une action de base")

    def travailler(self):
        """
        Méthode polymorphe : comportement de travail générique.
        Peut être redéfinie dans les sous-classes.
        """
        print(f"{self.nom} travaille de manière générique")
        self.action_speciale()


class RobotDomestique(Robots):
    """
    Robot destiné à exécuter des tâches ménagères.
    """

    def __init__(self, nom, couleur, taches_menageres):
        """
        Initialise un robot domestique avec une liste de tâches ménagères.
        """
        super().__init__(nom, couleur)
        self.__taches_menageres = taches_menageres
        self.__taches_effectuees = []

    @property
    def taches_menageres(self):
        """
        Retourne la liste des tâches ménagères possibles.
        """
        return self.__taches_menageres.copy()

    @property
    def taches_effectuees(self):
        """
        Retourne la liste des tâches déjà effectuées.
        """
        return self.__taches_effectuees.copy()

    def nettoyer(self, zone):
        """
        Effectue un nettoyage dans une zone.
        """
        self.energie = self.energie - 20
        self.__taches_effectuees.append(f"nettoyage {zone}")
        print(f"{self.nom} a nettoyé {zone}")
            
    def ranger(self, zone):
        """
        Effectue un rangement dans une zone.
        """
        self.energie = self.energie - 20
        self.__taches_effectuees.append(f"rangement {zone}")
        print(f"{self.nom} a rangé {zone}")

    def move(self, x, y):
        """
        Déplace le robot domestique avec un message contextuel.
        """
        print(f"{self.nom} (robot domestique) réfléchit avant de se déplacer...")
        super().move(x, y)

    def show(self):
        """
        Affiche les informations du robot domestique, y compris les tâches.
        """
        super().show()
        print(f"Tâches ménagères: {', '.join(self.taches_menageres)}")
        print(f"Tâches effectuées: {len(self.__taches_effectuees)}")

    def action_speciale(self):
        """
        Action spéciale propre aux robots domestiques.
        """
        print(f"{self.nom} range et organise l'espace de travail")

    def travailler(self):
        """
        Effectue toutes les tâches ménagères si l'énergie est suffisante.
        """
        print(f"{self.nom} commence ses tâches ménagères")
        for tache in self.__taches_menageres:
            if self.energie < 20:
                print(f"{self.nom} n'a pas assez d'énergie pour continuer les tâches ménagères")
                break
            
            # Choisie une zone aléatoire pour la tâche
            zone = random.choice([zone for zone in ["salon", "cuisine", "chambre", "salle de bain"] if tache+" "+zone not in self.__taches_effectuees])
            
            if tache == "nettoyage":
                self.nettoyer(zone)
            elif tache == "rangement":
                self.ranger(zone)
            else:
                print(f"{self.nom} ne sait pas effectuer la tâche: {tache}")
        
        self.action_speciale()
class RobotIndustriel(Robots):
    """
    Robot conçu pour des tâches industrielles de manutention.
    """

    def __init__(self, nom, couleur, capacite_charge):
        """
        Initialise un robot industriel avec une capacité de charge.
        """
        super().__init__(nom, couleur)
        self.__capacite_charge = capacite_charge
        self.__charge_actuelle = 0

    @property
    def capacite_charge(self):
        """
        Retourne la capacité maximale de charge du robot.
        """
        return self.__capacite_charge

    @capacite_charge.setter
    def capacite_charge(self, valeur):
        """
        Modifie la capacité de charge si la valeur est positive.
        """
        if valeur > 0:
            self.__capacite_charge = valeur
        else:
            print("Erreur: La capacité de charge doit être positive")

    @property
    def charge_actuelle(self):
        """
        Retourne le poids actuellement soulevé par le robot.
        """
        return self.__charge_actuelle

    def soulever(self, poids):
        """
        Soulève une charge si elle est dans la capacité et que l'énergie est suffisante.
        """
        if poids <= self.__capacite_charge and self.energie >= 25:
            self.__charge_actuelle = poids
            self.energie = self.energie - 25
            print(f"{self.nom} a soulevé {poids} kg")
        elif poids > self.__capacite_charge:
            print(f"{self.nom} ne peut pas soulever {poids} kg (capacité max: {self.__capacite_charge} kg)")
        else:
            print(f"{self.nom} n'a pas assez d'énergie pour soulever")

    def deposer(self):
        """
        Dépose la charge actuelle si le robot en porte une.
        """
        if self.__charge_actuelle > 0:
            print(f"{self.nom} a déposé {self.__charge_actuelle} kg")
            self.__charge_actuelle = 0
        else:
            print(f"{self.nom} ne porte rien")

    def move(self, x, y):
        """
        Déplace le robot industriel, plus lentement s’il porte une charge.
        """
        if self.charge_actuelle > 0:
            print(f"{self.nom} (robot industriel) se déplace lentement avec une charge.")
        else:
            print(f"{self.nom} (robot industriel) se déplace rapidement.")
        super().move(x, y)

    def show(self):
        """
        Affiche les informations du robot industriel, y compris la charge.
        """
        super().show()
        print(f"Capacité de charge: {self.__capacite_charge} kg")
        print(f"Charge actuelle: {self.__charge_actuelle} kg")

    def action_speciale(self):
        """
        Action spéciale propre aux robots industriels.
        """
        print(f"{self.nom} calibre ses capteurs de poids et vérifie ses systèmes hydrauliques")

    def travailler(self):
        """
        Effectue une tâche de manutention en soulevant et déposant une charge.
        """
        print(f"{self.nom} commence ses opérations industrielles")
        poids = random.randint(10, self.__capacite_charge)
        self.soulever(poids)
        if self.__charge_actuelle > 0:
            self.move(random.randint(1, 10), random.randint(1, 10))
            self.deposer()
        self.action_speciale()