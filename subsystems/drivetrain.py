import math
from math import pi

from typing import Callable

from commands2 import Subsystem, Command
from commands2.sysid import SysIdRoutine

from wpilib import (
    Mechanism2d,
    Timer,
    Color8Bit,
    SmartDashboard,
    RobotBase,
    DriverStation,
    RobotController,
)
from wpilib.sysid import SysIdRoutineLog

from wpimath.system.plant import DCMotor
from wpimath.units import (
    amperes,
    meters,
    inchesToMeters,
    seconds,
    meters_per_second,
    feetToMeters,
    degrees_per_second,
)
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import (
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    ChassisSpeeds,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath import applyDeadband

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

from rev import SparkMax, SparkMaxConfig, SparkMaxSim

from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration
from phoenix6.signals import SensorDirectionValue
from phoenix6 import SignalLogger, swerve, units, utils

from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

from navx import AHRS
import navx


class _SwerveModuleConfiguration:
    def __init__(
        self,
        drive_id: int,
        turn_id: int,
        cancoder_id: int,
        name: str,
        center_offset: Translation2d,
        zero_offset: float = 0.0,
        drive_inverted: bool = False,
        turn_inverted: bool = False,
    ):
        self.drive_id = drive_id
        self.turn_id = turn_id
        self.cancoder_id = cancoder_id
        self.name = name
        self.center_offset = center_offset
        self.zero_offset = zero_offset
        self.drive_inverted = drive_inverted
        self.turn_inverted = turn_inverted


class _SwerveModule(Subsystem):
    # A single swerve module consisting of a drive motor controlled by a Spark Max, turn motor on a Spark Max, and cancoder.
    # Default configuration values are based on the MK4i swerve module from Swerve Drive Specialties on an L1 configuration

    idle_mode = SparkMaxConfig.IdleMode.kBrake
    # or      = SparkMaxConfig.IdleMode.kCoast

    # PID coefficients
    ## drive (velocity)
    driveP: float = 0.0001
    driveI: float = 0.0
    driveD: float = 0.00
    ### drive velocityfeedforward coefficient
    driveF: float = 0.00125

    ## turn (position)
    turnP: float = 1.5
    turnI: float = 0.0
    turnD: float = 0.01

    # drive configs
    drive_current_limit: amperes = 30 if RobotBase.isReal() else 60
    ## https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    gear_ratio: float = 8.14
    wheel_radius: meters = inchesToMeters(2)
    max_velocity: meters_per_second = feetToMeters(12.5)
    drive_max_rps: float = DCMotor.NEO(1).freeSpeed / (2 * pi)

    # turn configs
    turn_current_limit: amperes = 20 if RobotBase.isReal() else 60
    ## https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    turn_gear_ratio: float = 150.0 / 7.0
    min_stop_time_for_reset: seconds = 0.25
    turn_reset_speed_threshold: degrees_per_second = 5.0
    turn_max_rps: float = DCMotor.NEO(1).freeSpeed / (2 * pi)

    @staticmethod
    def from_config(config: _SwerveModuleConfiguration) -> "_SwerveModule":
        return _SwerveModule(
            drive_id=config.drive_id,
            turn_id=config.turn_id,
            cancoder_id=config.cancoder_id,
            name=config.name,
            zero_offset=config.zero_offset,
            drive_inverted=config.drive_inverted,
            turn_inverted=config.turn_inverted,
        )

    def __init__(
        self,
        drive_id: int,
        turn_id: int,
        cancoder_id: int,
        name: str,
        zero_offset: float = 0.0,
        drive_inverted: bool = False,
        turn_inverted: bool = False,
    ) -> None:
        super().__init__()

        self.nettable = NetworkTableInstance.getDefault().getTable(
            f"000SwerveModules/{name}"
        )

        drive_config = SparkMaxConfig()
        drive_config.closedLoop.P(self.driveP).I(self.driveI).D(self.driveD).velocityFF(
            self.driveF
        )
        drive_config.inverted(drive_inverted)
        drive_config.setIdleMode(self.idle_mode)
        drive_config.smartCurrentLimit(self.drive_current_limit)
        drive_config.encoder.velocityConversionFactor(
            2
            * pi
            * self.wheel_radius
            / (self.gear_ratio * 60)  # to be measured in meters/second
        ).positionConversionFactor(
            2 * pi * self.wheel_radius / self.gear_ratio  # to be measured in meters
        )

        turn_config = SparkMaxConfig()
        turn_config.closedLoop.P(self.turnP).I(self.turnI).D(
            self.turnD
        ).positionWrappingEnabled(True).positionWrappingInputRange(0, 180.0)
        turn_config.inverted(turn_inverted)
        turn_config.setIdleMode(self.idle_mode)
        turn_config.smartCurrentLimit(self.turn_current_limit)
        turn_config.encoder.positionConversionFactor(
            360.0
            / self.turn_gear_ratio
            # to be measured in degrees
        ).velocityConversionFactor(
            60.0
            / self.turn_gear_ratio
            # to be measured in degrees/second
        )

        cancoder_config = CANcoderConfiguration()
        cancoder_config.magnet_sensor.with_absolute_sensor_discontinuity_point(
            0.5
        ).with_magnet_offset(zero_offset).with_sensor_direction(
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )

        self.drive_motor = SparkMax(drive_id, SparkMax.MotorType.kBrushless)
        self.turn_motor = SparkMax(turn_id, SparkMax.MotorType.kBrushless)
        self.cancoder = CANcoder(cancoder_id)
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getEncoder()

        self.drive_motor.configure(
            drive_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.turn_motor.configure(
            turn_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.cancoder.configurator.apply(cancoder_config)

        self.drive_closedloop = self.drive_motor.getClosedLoopController()
        self.turn_closedloop = self.turn_motor.getClosedLoopController()
        self.get_cancoder_position = self.cancoder.get_absolute_position().as_supplier()

        self.turn_reset_timer = Timer()
        self.turn_reset_timer.start()

        self.drive_sim = SparkMaxSim(self.drive_motor, DCMotor.NEO(1))
        self.drive_encoder_sim = self.drive_sim.getRelativeEncoderSim()
        self.turn_sim = SparkMaxSim(self.turn_motor, DCMotor.NEO(1))

        mech = Mechanism2d(100, 100)
        root = mech.getRoot(self.getName(), 50, 50)
        self.setpoint_mech = root.appendLigament(
            "setpoint",
            5,
            self.get_angle().degrees(),
            color=Color8Bit(0, 0, 255),
        )
        self.actual_mech = root.appendLigament(
            "real",
            5,
            self.get_angle().degrees(),
            color=Color8Bit(255, 200, 0),
        )

        self.setpoint = SwerveModuleState(0, Rotation2d())

        self.setpoint_publisher = self.nettable.getStructTopic(
            "setpoint", SwerveModuleState
        ).publish()

        self.state_publisher = self.nettable.getStructTopic(
            "state", SwerveModuleState
        ).publish()

        SmartDashboard.putData(f"SwerveModule/{name}", mech)

    def periodic(self) -> None:
        # do swervemodule specific periodic tasks to optimize driving
        self.setpoint.optimize(self.get_angle())
        # not sure why this does not work:
        self.setpoint.cosineScale(self.get_angle())
        # self.setpoint.speed *= (self.setpoint.angle - self.get_angle()).cos()

        # log inputs
        self.setpoint_publisher.set(self.setpoint)
        self.state_publisher.set(self.get_state())

        self.setpoint_mech.setAngle(self.setpoint.angle.degrees() + 90)
        self.setpoint_mech.setLength(5 + 50 * self.setpoint.speed / self.max_velocity)

        # interact with hardware
        self.drive_closedloop.setReference(
            self.setpoint.speed, SparkMax.ControlType.kVelocity
        )
        self.turn_closedloop.setReference(
            self.setpoint.angle.degrees(), SparkMax.ControlType.kPosition
        )

        if abs(self.get_turn_velocity().degrees()) < self.turn_reset_speed_threshold:
            self.turn_reset_timer.start()
        else:
            self.turn_reset_timer.reset()
            self.turn_reset_timer.stop()
        if (
            self.turn_reset_timer.hasElapsed(self.min_stop_time_for_reset)
            and RobotBase.isReal()
        ):
            # reset turn motor position to absolute cancoder position
            self.turn_motor.getEncoder().setPosition(self.get_cancoder_position())

        # log outputs
        self.actual_mech.setAngle(self.get_angle().degrees() + 90)
        self.actual_mech.setLength(5 + 50 * self.get_velocity() / self.max_velocity)
        # self.actual_mech.setLength(50)

    def simulationPeriodic(self) -> None:
        self.drive_sim.iterate(
            self.drive_motor.getAppliedOutput() * self.drive_max_rps * self.gear_ratio,
            12,
            0.02,
        )
        self.turn_sim.iterate(
            self.turn_motor.getAppliedOutput()
            * self.turn_max_rps
            * self.turn_gear_ratio,
            12,
            0.02,
        )

        self.cancoder.sim_state.set_raw_position(self.turn_encoder.getPosition() / 360)

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.turn_encoder.getPosition())

    def get_turn_velocity(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.turn_encoder.getVelocity())

    def get_velocity(self) -> meters_per_second:
        return self.drive_encoder.getVelocity()

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_velocity(), self.get_angle())

    def get_setpoint(self) -> SwerveModuleState:
        return self.setpoint

    def get_pose(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_encoder.getPosition(), self.get_angle())

    def set_setpoint(self, state: SwerveModuleState) -> None:
        self.setpoint = state


class Drivetrain(Subsystem):
    _gyro_comm_type = navx.AHRS.NavXComType.kUSB1
    _offset: meters = inchesToMeters(
        11.5
    )  # half the robot width/length for a square robot
    fl_config: _SwerveModuleConfiguration = _SwerveModuleConfiguration(
        drive_id=1,
        turn_id=2,
        cancoder_id=11,
        name="fl",
        zero_offset=0.0,
        drive_inverted=False,
        turn_inverted=False,
        center_offset=Translation2d(_offset, _offset),
    )

    fr_config: _SwerveModuleConfiguration = _SwerveModuleConfiguration(
        drive_id=3,
        turn_id=4,
        cancoder_id=12,
        name="fr",
        zero_offset=0.0,
        drive_inverted=False,
        turn_inverted=False,
        center_offset=Translation2d(_offset, -_offset),
    )

    bl_config: _SwerveModuleConfiguration = _SwerveModuleConfiguration(
        drive_id=5,
        turn_id=6,
        cancoder_id=13,
        name="bl",
        zero_offset=0.0,
        drive_inverted=False,
        turn_inverted=False,
        center_offset=Translation2d(-_offset, _offset),
    )

    br_config: _SwerveModuleConfiguration = _SwerveModuleConfiguration(
        drive_id=7,
        turn_id=8,
        cancoder_id=14,
        name="br",
        zero_offset=0.0,
        drive_inverted=False,
        turn_inverted=False,
        center_offset=Translation2d(-_offset, -_offset),
    )

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)

    _curr_alliance: DriverStation.Alliance | None = None

    # the real class from phoenix6 has far more configuration options, but we don't need to do that for this robot
    def __init__(
        self,
    ) -> None:
        self.nettable = NetworkTableInstance.getDefault().getTable("000Drivetrain")

        self.pub_pose = self.nettable.getStructTopic("pose", Pose2d).publish()

        self._has_applied_operator_perspective: bool = False
        self.gyro_offset: Rotation2d = Rotation2d()

        self.setpoint_states = [
            SwerveModuleState(0, Rotation2d()),
            SwerveModuleState(0, Rotation2d()),
            SwerveModuleState(0, Rotation2d()),
            SwerveModuleState(0, Rotation2d()),
        ]

        self._fl = _SwerveModule.from_config(self.fl_config)

        self._fr = _SwerveModule.from_config(self.fr_config)

        self._bl = _SwerveModule.from_config(self.bl_config)

        self._br = _SwerveModule.from_config(self.br_config)

        self.gyro = AHRS(self._gyro_comm_type)

        self.kinematics = SwerveDrive4Kinematics(
            self.fl_config.center_offset,
            self.fr_config.center_offset,
            self.bl_config.center_offset,
            self.br_config.center_offset,
        )

        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self._get_gyro_angle(),
            (
                self._fl.get_pose(),
                self._fr.get_pose(),
                self._bl.get_pose(),
                self._br.get_pose(),
            ),
            Pose2d(),
        )

        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotations_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """ find the motor stuff for translation """

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        self._configure_auto_builder()

    def periodic(self):
        self.nettable.putNumber("Gyro Off", self.gyro_offset.degrees())
        self.nettable.putNumber(
            "Believed angle", self.odometry.getEstimatedPosition().rotation().degrees()
        )
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True
                self._curr_alliance = alliance_color

        self.odometry.update(
            self._get_gyro_angle()
            + (
                self.gyro_offset
                if self._has_applied_operator_perspective
                else Rotation2d()
            ),
            (
                self._fl.get_pose(),
                self._fr.get_pose(),
                self._bl.get_pose(),
                self._br.get_pose(),
            ),
        )

        self.pub_pose.set(self.odometry.getEstimatedPosition())

        self._fl.set_setpoint(self.setpoint_states[0])
        self._fr.set_setpoint(self.setpoint_states[1])
        self._bl.set_setpoint(self.setpoint_states[2])
        self._br.set_setpoint(self.setpoint_states[3])

    def simulationPeriodic(self) -> None:
        omega = self.kinematics.toChassisSpeeds(
            (
                self._fl.get_state(),
                self._fr.get_state(),
                self._bl.get_state(),
                self._br.get_state(),
            )
        ).omega
        self.gyro_offset += Rotation2d(omega * 0.02)

    def _configure_auto_builder(self) -> None:
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            lambda: self.get_state().pose,
            self.reset_pose,
            lambda: self.get_state().speeds,
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds.with_speeds(speeds)
                .with_wheel_force_feedforwards_x(
                    feedforwards.robotRelativeForcesXNewtons
                )
                .with_wheel_force_feedforwards_y(
                    feedforwards.robotRelativeForcesYNewtons
                )
            ),
            PPHolonomicDriveController(
                PIDConstants(
                    7,
                    0,
                    0,
                ),
                PIDConstants(
                    7,
                    0,
                    0,
                ),
            ),
            config,
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue)
            == DriverStation.Alliance.kRed,
            self,
        )

    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def reset_pose(self, pose: Pose2d) -> None:
        self.odometry.resetPose(pose)

    def set_operator_perspective_forward(self, robot_forward_rotation: Rotation2d):
        self.gyro_offset = robot_forward_rotation

    def get_state(
        self,
    ) -> swerve.swerve_drivetrain.SwerveDrivetrain.SwerveDriveState:
        state = swerve.swerve_drivetrain.SwerveDrivetrain.SwerveDriveState()
        state.module_targets = list(self.setpoint_states)
        state.module_positions = [
            self._fl.get_pose(),
            self._fr.get_pose(),
            self._bl.get_pose(),
            self._br.get_pose(),
        ]
        state.module_states = [
            self._fl.get_state(),
            self._fr.get_state(),
            self._bl.get_state(),
            self._br.get_state(),
        ]
        state.odometry_period = 0.02  # seconds
        state.pose = self.odometry.getEstimatedPosition()
        state.raw_heading = self._get_gyro_angle()
        state.speeds = self.kinematics.toChassisSpeeds(
            (
                state.module_states[0],
                state.module_states[1],
                state.module_states[2],
                state.module_states[3],
            ),
        )
        return state

    def add_vision_measurement(
        self,
        vision_robot_pose: Pose2d,
        timestamp: units.second,
        vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ):
        """
        Adds a vision measurement to the Kalman Filter. This will correct the
        odometry pose estimate while still accounting for measurement noise.

        Note that the vision measurement standard deviations passed into this method
        will continue to apply to future measurements until a subsequent call to
        set_vision_measurement_std_devs or this method.

        :param vision_robot_pose:           The pose of the robot as measured by the vision camera.
        :type vision_robot_pose:            Pose2d
        :param timestamp:                   The timestamp of the vision measurement in seconds.
        :type timestamp:                    second
        :param vision_measurement_std_devs: Standard deviations of the vision pose measurement
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians.
        :type vision_measurement_std_devs:  tuple[float, float, float] | None
        """
        if vision_measurement_std_devs is not None:
            self.odometry.addVisionMeasurement(
                vision_robot_pose,
                timestamp,
                vision_measurement_std_devs,
            )
        else:
            self.odometry.addVisionMeasurement(
                vision_robot_pose,
                timestamp,
            )

    def get_velocity(self) -> ChassisSpeeds:
        """
        Gets the current velocity of the robot.

        :returns: The current velocity of the robot.
        :rtype: ChassisSpeeds
        """
        return self.kinematics.toChassisSpeeds(
            (
                self._fl.get_state(),
                self._fr.get_state(),
                self._bl.get_state(),
                self._br.get_state(),
            )
        )

    def _get_gyro_angle(self) -> Rotation2d:
        """
        Gets the robot's gyro angle.

        :returns: The robot's gyro angle.
        :rtype: Rotation2d
        """
        return Rotation2d.fromDegrees(self.gyro.getYaw())

    def seed_field_centric(self) -> None:
        self.gyro.zeroYaw()

    def set_control(self, request: swerve.requests.SwerveRequest) -> None:
        """
        Sets the control request for the drivetrain.

        :param request: The control request to set.
        :type request: swerve.requests.SwerveRequest
        """
        if isinstance(request, swerve.requests.Idle):
            ...  # does nothing
        elif isinstance(request, swerve.requests.SwerveDriveBrake):
            self.setpoint_states = [
                SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            ]
        elif isinstance(request, swerve.requests.FieldCentric):
            alliance_mult = 1 + 0 * (
                -1
                if RobotBase.isSimulation()
                and self._curr_alliance == DriverStation.Alliance.kRed
                else 1
            )
            self.setpoint_states = self.kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    applyDeadband(request.velocity_x, request.deadband) * alliance_mult,
                    applyDeadband(request.velocity_y, request.deadband) * alliance_mult,
                    applyDeadband(request.rotational_rate, request.rotational_deadband),
                    self.get_state().pose.rotation(),
                ),
                centerOfRotation=request.center_of_rotation,
            )
        elif isinstance(request, swerve.requests.RobotCentric):
            self.setpoint_states = self.kinematics.toSwerveModuleStates(
                ChassisSpeeds(
                    applyDeadband(request.velocity_x, request.deadband),
                    applyDeadband(request.velocity_y, request.deadband),
                    applyDeadband(request.rotational_rate, request.rotational_deadband),
                ),
                centerOfRotation=request.center_of_rotation,
            )
        elif isinstance(request, swerve.requests.PointWheelsAt):
            self.setpoint_states = [
                SwerveModuleState(0, request.module_direction),
                SwerveModuleState(0, request.module_direction),
                SwerveModuleState(0, request.module_direction),
                SwerveModuleState(0, request.module_direction),
            ]
        elif isinstance(request, swerve.requests.ApplyRobotSpeeds) or isinstance(
            request, swerve.requests.ApplyFieldSpeeds
        ):
            self.setpoint_states = self.kinematics.toSwerveModuleStates(request.speeds)
        elif isinstance(request, swerve.requests.FieldCentricFacingAngle):
            controller = request.heading_controller
            self.setpoint_states = self.kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    applyDeadband(request.velocity_x, request.deadband),
                    applyDeadband(request.velocity_y, request.deadband),
                    controller.calculate(
                        self.get_state().pose.rotation().radians(),
                        request.target_direction.radians(),
                        Timer().getFPGATimestamp(),
                    ),
                    self.get_state().pose.rotation(),
                ),
                centerOfRotation=request.center_of_rotation,
            )
        else:
            raise NotImplementedError(
                f"Drivetrain.set_control does not (yet?) support request of type {type(request)}"
            )


class TunerConstants:
    @staticmethod
    def create_drivetrain() -> Drivetrain:
        return Drivetrain()
