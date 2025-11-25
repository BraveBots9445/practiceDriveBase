from wpilib import SmartDashboard

from wpimath.units import degreesToRadians

from ntcore.util import ntproperty

from commands2 import Command
from commands2.button import CommandXboxController

from pathplannerlib.auto import AutoBuilder

from phoenix6.swerve.requests import FieldCentric

from subsystems import Drivetrain, Vision


class RobotContainer:
    max_vel = ntproperty("/000Drivetrain/0max_vel_m", 3.0)  # meters per second
    max_rot = ntproperty(
        "/000Drivetrain/0max_rotation_deg", 360.0
    )  # degrees per second

    def __init__(self):
        self.joystick = CommandXboxController(0)
        self.drivetrain = Drivetrain()
        self.vision = Vision(
            self.drivetrain.add_vision_measurement,
            lambda: self.drivetrain.get_state().pose,
            self.drivetrain.get_velocity,
        )
        self.chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.chooser)

    def set_teleop_bindings(self):
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: FieldCentric()
                .with_velocity_x(self.joystick.getLeftY() * self.max_vel)
                .with_velocity_y(self.joystick.getLeftX() * self.max_vel)
                .with_rotational_rate(
                    -self.joystick.getRightX() * degreesToRadians(self.max_rot)
                )
                .with_deadband(0.1 * self.max_vel)
                .with_rotational_deadband(0.1 * degreesToRadians(self.max_rot))
            )
        )

    def get_auto_command(self) -> Command:
        return self.chooser.getSelected()
