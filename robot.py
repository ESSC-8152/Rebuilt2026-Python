import commands2

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.autonomousCommand = None

    def autonomousInit(self):
        self.autonomousCommand = self.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def getAutonomousCommand(self):
        return None

if __name__ == "__main__":
    commands2.runRobot(MyRobot)
