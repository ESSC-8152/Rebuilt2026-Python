import commands2
from commands2.timedcommandrobot import seconds
from wpilib import TimedRobot

from robotcontainer import RobotContainer

# Instructions pour deploy (ne pas appeler wpilib.run ici):
#   Deploy:      py -3.13 -m robotpy deploy
# Dans un environment virtuel:
#   Deploy:      robotpy deploy

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.m_robotContainer = RobotContainer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

if __name__ == "__main__":
    # Ne pas lancer ce ficier pour deploy le code
    import sys
    print("[INFO] pour lancer le robot")
    print("Utilise la commandes:")
    print(" py -3.13 -m robotpy deploy")
    sys.exit(1)
