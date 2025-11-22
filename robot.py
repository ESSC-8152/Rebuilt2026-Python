import commands2
from robotcontainer import RobotContainer

# Launch instructions (do not call wpilib.run here):
#   Simulation:  py -3.12 -m robotpy sim
#   Deploy:      py -3.12 -m robotpy deploy
# In a virtualenv you can use the `robotpy` command directly.

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.m_robotContainer = RobotContainer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

if __name__ == "__main__":
    # Inform the user they invoked the file directly instead of using the recommended launcher
    import sys
    print("[INFO] Ce script ne lance plus le robot directement.")
    print("Utilise plutôt les commandes suivantes:")
    print("  Simulation : py -3.12 -m robotpy sim")
    print("  Déploiement : py -3.12 -m robotpy deploy")
    if len(sys.argv) > 1:
        print(f"Arguments ignorés: {sys.argv[1:]}")
    sys.exit(1)
