from wpilib import TimedRobot, Joystick, SmartDashboard
from commands2 import Command, CommandScheduler

from subsystems import Drivetrain, Vision

from ntcore.util import ntproperty

from phoenix6.swerve.requests import FieldCentric

from pathplannerlib.auto import AutoBuilder


class Robot(TimedRobot):
    max_vel = ntproperty("/Robot/max_vel", 3.0)  # meters per second
    max_rot_rate = ntproperty("/Robot/max_rot_rate", 3.0)

    def robotInit(self) -> None:
        self.joystick = Joystick(0)
        self.joystick.setTwistChannel(4)
        self.drivetrain = Drivetrain()
        self.vision = Vision(
            self.drivetrain.add_vision_measurement,
            lambda: self.drivetrain.get_state().pose,
            self.drivetrain.get_velocity,
        )
        self.chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.chooser)

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    # def teleopInit(self) -> None:
    #     self.mod.set_setpoint(SwerveModuleState(1, Rotation2d.fromDegrees(0)))

    def autonomousInit(self) -> None:
        c = self.chooser.getSelected()
        if isinstance(c, Command):
            c.schedule()

    def teleopPeriodic(self) -> None:
        self.drivetrain.set_control(
            FieldCentric()
            .with_velocity_x(self.joystick.getY() * self.max_vel)
            .with_velocity_y(self.joystick.getX() * self.max_vel)
            .with_rotational_rate(-self.joystick.getTwist() * self.max_rot_rate)
            .with_deadband(0.1 * self.max_vel)
            .with_rotational_deadband(0.1 * self.max_rot_rate)
        )
