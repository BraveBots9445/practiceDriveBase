from wpilib import TimedRobot
from commands2 import Command, CommandScheduler

from robotcontainer import RobotContainer


class Robot(TimedRobot):
    autocommand: Command

    def robotInit(self) -> None:
        self.robotcontainer = RobotContainer()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    # def teleopInit(self) -> None:
    #     self.mod.set_setpoint(SwerveModuleState(1, Rotation2d.fromDegrees(0)))

    def autonomousInit(self) -> None:
        self.autocommand = self.robotcontainer.get_auto_command()
        if self.autocommand is not None:
            self.autocommand.schedule()

    def autonomousExit(self) -> None:
        if self.autocommand is not None and self.autocommand.isScheduled():
            self.autocommand.cancel()

    def teleopInit(self) -> None:
        self.robotcontainer.set_teleop_bindings()
