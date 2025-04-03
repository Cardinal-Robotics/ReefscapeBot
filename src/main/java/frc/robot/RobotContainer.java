// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.Constants.AlgaeMechanismConstants;
import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SpinnyheheboiSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SimulatedGame;
import frc.robot.commands.AlignAprilTag;
import frc.robot.commands.DriveToInteractionArea;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;

import swervelib.SwerveInputStream;

public class RobotContainer {
    // Misc.
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private Optional<Command> m_interactionAreaDriveCommand = Optional.empty();
    public static InteractionState interactionState = InteractionState.Coral;
    private final SendableChooser<Command> m_autoChooser;

    public enum InteractionState {
        Coral,
        Algae
    }
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Controllers
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Subsystems
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(null, null);
    private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();

    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveDrive.getLibSwerveDrive());
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem(m_elevatorSubsystem);
    private final CoralSubsystem m_coralSubsystem = new CoralSubsystem(m_elevatorSubsystem);
    private final SpinnyheheboiSubsytem m_SpinnyheheboiSubsytem = new SpinnyheheboiSubsytem();
    // private final LightSubsystem m_lightSubsystem = new
    // LightSubsystem(m_elevatorSubsystem);
    // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

    private final SimulatedGame m_gameSim = new SimulatedGame(m_elevatorSubsystem, m_algaeSubsystem, m_coralSubsystem,
            m_swerveDrive);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // YAGSL Swerve input streams
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SwerveInputStream m_driveInputStream = SwerveInputStream.of(m_swerveDrive.getLibSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerHeadingAxis( // Maps joystick rotation to rotation on field. So if the joystick goes bottom
                                        // right, the robot rotates to the bottom red from the perspective of your
                                        // alliance
                    () -> m_driverController.getRightX(),
                    () -> m_driverController.getRightY())
            .headingWhile(true)
            .deadband(OperatorConstants.kDeadband) // The joystick has to exceed the deadband for it
                                                   // to register. This way slight micro-movements doesn't suddenly move
                                                   // the robot.
            .scaleTranslation(0.8) // If the joystick goes to 100%, this scales it down to 80%.
            .scaleRotation(0.8)
            .allianceRelativeControl(true); // Field orientation flips to be on the your team's side.
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Commands
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final Command m_driveFieldOriented = m_swerveDrive
            .driveFieldOriented(m_driveInputStream);
    private final Command m_resetGyro = Commands.runOnce(() -> m_swerveDrive.resetGyro(), m_swerveDrive);

    private final DriveToInteractionArea m_driveToInteractionArea = new DriveToInteractionArea(m_swerveDrive);
    private final AlignAprilTag m_alignAprilTag = new AlignAprilTag(m_visionSubsystem, m_swerveDrive,
            m_elevatorSubsystem);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    public RobotContainer() {
        var leftCamera = new HttpCamera("left", "http://photonvision.local:1182/stream.mjpg");
        var rightCamera = new HttpCamera("right", "http://photonvision.local:1184/stream.mjpg");
        CameraServer.startAutomaticCapture(leftCamera);
        CameraServer.startAutomaticCapture(rightCamera);

        DriverStation.silenceJoystickConnectionWarning(true);
        m_elevatorSubsystem.setCoralSubsystem(m_coralSubsystem);
        m_elevatorSubsystem.setScaleDriverInputConsumer((Double scale) -> {
            m_driveInputStream.scaleTranslation(scale);

            m_swerveDrive.getLibSwerveDrive().swerveController.config.maxAngularVelocity = (4 * Math.PI) * scale;
        });

        registerNamedCommands();
        configureBindings();

        m_swerveDrive.setDefaultCommand(m_driveFieldOriented);

        m_autoChooser = AutoBuilder.buildAutoChooser("Leave");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        PathPlannerLogging.setLogActivePathCallback((List<Pose2d> trajectory) -> {
            m_swerveDrive.getLibSwerveDrive().field.getObject("trajectory").setPoses(trajectory);
        });

        // m_coralsubsystem.setDefaultCommand(m_coralCommand);
        /*
         * m_lightSubsystem.setDefaultCommand(
         * DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
         * ? m_lightSubsystem.setConstantColor(253, 11, 205)
         * : m_lightSubsystem.setConstantColor(0, 0, 255));
         */
    }

    private void registerNamedCommands() {
        // AprilTag Alignment

        NamedCommands.registerCommand("AprilTagAlignRight",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, m_elevatorSubsystem, 0.5, 0.15));
        NamedCommands.registerCommand("AprilTagAlignLeft",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, m_elevatorSubsystem, 0.5, -0.15));

        // Elevator positions
        NamedCommands.registerCommand("ElevatorCoralIntake",
                m_elevatorSubsystem.setElevatorGoalCommand(ElevatorTarget.CoralIntake));
        NamedCommands.registerCommand("ElevatorCoralL1", m_elevatorSubsystem.setElevatorGoalCommand(ElevatorTarget.L1));
        NamedCommands.registerCommand("ElevatorCoralL2", m_elevatorSubsystem.setElevatorGoalCommand(ElevatorTarget.L2));
        NamedCommands.registerCommand("ElevatorCoralL3", m_elevatorSubsystem.setElevatorGoalCommand(ElevatorTarget.L3));
        NamedCommands.registerCommand("ElevatorCoralL4", m_elevatorSubsystem.setElevatorGoalCommand(ElevatorTarget.L4));

        // Coral mechanism
        NamedCommands.registerCommand("CoralTiltIntake",
                m_coralSubsystem.setTargetCommand(CoralMechanismConstants.kTargetAngleIntake));
        NamedCommands.registerCommand("CoralTiltStore",
                m_coralSubsystem.setTargetCommand(CoralMechanismConstants.kTargetAngleStore));
        NamedCommands.registerCommand("CoralTiltL1",
                m_coralSubsystem.setTargetCommand(CoralMechanismConstants.kTargetAngleL1));
        NamedCommands.registerCommand("CoralTiltL2-L3",
                m_coralSubsystem.setTargetCommand(CoralMechanismConstants.kTargetAngleL2_3));
        NamedCommands.registerCommand("CoralTiltL4",
                m_coralSubsystem.setTargetCommand(CoralMechanismConstants.kTargetAngleL4));

        NamedCommands.registerCommand("CoralRelease", m_coralSubsystem.setIntakeMotorCommand(-1, 2));
        NamedCommands.registerCommand("CoralIntake", m_coralSubsystem.setIntakeMotorCommand(0.2, 1));
    }

    private void configureBindings() {
        // Driver controls
        m_driverController.y().onTrue(m_resetGyro);
        m_driverController.rightBumper().whileTrue(m_swerveDrive.driveRelative(new Translation2d(0, -0.35)));
        m_driverController.leftBumper().whileTrue(m_swerveDrive.driveRelative(new Translation2d(0, 0.35)));
        m_driverController.rightTrigger().whileTrue(m_swerveDrive.driveRelative(new Translation2d(0.35, 0)));
        m_driverController.leftTrigger().whileTrue(m_swerveDrive.driveRelative(new Translation2d(-0.35, 0)));

        m_driverController.a().whileTrue(m_alignAprilTag);
        /*
         * m_driverController.b().and(
         * () -> 0 < 1)
         * .whileTrue(m_swerveDrive.driveToPose(new Pose2d(11.55, 7.5,
         * Rotation2d.kCCW_90deg)));
         */

        // m_driverController.povRight().onTrue(Commands.runOnce(() ->
        // m_alignAprilTag.setOffsetPose(-0.5, 0.15)));
        m_driverController.povRight()
                .whileTrue(new AlignAprilTag(m_visionSubsystem, m_swerveDrive, m_elevatorSubsystem, 0.5, 0.15));
        m_driverController.povLeft()
                .whileTrue(new AlignAprilTag(m_visionSubsystem, m_swerveDrive, m_elevatorSubsystem, 0.5, -0.15));
        m_driverController.povUp()
                .whileTrue(new AlignAprilTag(m_visionSubsystem, m_swerveDrive, m_elevatorSubsystem, 0.5, 0));
        /*
         * m_driverController.povDown().whileTrue(Commands.run(() -> {
         * Pose2d nearestPose =
         * m_swerveDrive.getPose().nearest(DriveConstants.kInteractionAreas);
         * double distance =
         * nearestPose.getTranslation().getDistance(m_swerveDrive.getPose().
         * getTranslation());
         * if (distance < 1)
         * m_swerveDrive.driveToPose(nearestPose).until(() ->
         * !m_driverController.povDown().getAsBoolean())
         * .schedule();
         * }));
         */

        /*
         * m_driverController.x()
         * .toggleOnTrue(Commands.runOnce(() ->
         * m_swerveDrive.setDefaultCommand(m_driveFieldOriented)));
         */

        // Climber controls
        /*
         * m_driverController.leftStick()
         * .onTrue(Commands.runOnce(() ->
         * m_climberSubsystem.setGoal(ClimberConstants.kCrushingFrame),
         * m_climberSubsystem));
         * m_driverController.rightStick()
         * .onTrue(Commands.runOnce(() ->
         * m_climberSubsystem.setGoal(ClimberConstants.kCrushingCage),
         * m_climberSubsystem));
         */

        /*
         * m_driverController.b()
         * .onTrue(m_swerveDrive.driveToPose(DriverStation.getAlliance().get() ==
         * Alliance.Red
         * ? Constants.DriveConstants.kInitialRedRobotPose
         * : Constants.DriveConstants.kInitialBlueRobotPose));
         */

        // Operator controls

        // State Controls
        m_operatorController.rightStick()
                .onTrue(Commands.runOnce(() -> {
                    interactionState = InteractionState.Algae;
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleStore);
                }));

        m_operatorController.leftStick()
                .onTrue(Commands.runOnce(() -> {
                    interactionState = InteractionState.Coral;
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetDisabledAngle);
                }));

        // Coral Controls
        m_operatorController.leftBumper()
                .onTrue(m_coralSubsystem.setIntakeMotorCommand(0.5))
                .onFalse(m_coralSubsystem.setIntakeMotorCommand(0));
        m_operatorController.rightBumper()
                .onTrue(m_coralSubsystem.setIntakeMotorCommand(() -> ElevatorSubsystem.coralReleaseSpeed))
                .onFalse(m_coralSubsystem.setIntakeMotorCommand(0))
                .onFalse(Commands.runOnce(() -> {
                    Optional<PhotonTrackedTarget> target = Robot.isSimulation() ? m_visionSubsystem.getClosestTarget()
                            : m_visionSubsystem.getBestTarget();
                    if (target.isEmpty())
                        return;

                    Optional<Transform2d> potentialPose = m_visionSubsystem
                            .getRobotPoseRelativeToAprilTag(target.get().fiducialId);

                    if (potentialPose.isEmpty())
                        return;

                    Logger.recordOutput("Offset", potentialPose.get());
                }));

        // Algae Controls
        m_operatorController.leftTrigger()
                .onTrue(m_algaeSubsystem.spinIntakeMotorCommand(-1))
                .onFalse(m_algaeSubsystem.spinIntakeMotorCommand(-0.1));

        m_operatorController.rightTrigger()
                .onTrue(m_algaeSubsystem.spinIntakeMotorCommand(1))
                .onFalse(m_algaeSubsystem.spinIntakeMotorCommand(0))
                .onFalse(Commands.runOnce(() -> {
                    Optional<PhotonTrackedTarget> target = Robot.isSimulation() ? m_visionSubsystem.getClosestTarget()
                            : m_visionSubsystem.getBestTarget();
                    if (target.isEmpty())
                        return;

                    Optional<Transform2d> potentialPose = m_visionSubsystem
                            .getRobotPoseRelativeToAprilTag(target.get().fiducialId);

                    if (potentialPose.isEmpty())
                        return;

                    Logger.recordOutput("Offset", potentialPose.get());
                }));

        // Elevator Positions
        m_operatorController.button(7)
                .onTrue(Commands.runOnce(() -> {
                    // m_lightSubsystem.elevatorPattern();
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.CoralIntake);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleIntake);
                }));

        m_operatorController.button(8).and(() -> m_elevatorSubsystem.getPosition() < 0.1)
                .onTrue(Commands.runOnce(() -> m_elevatorSubsystem.resetEncoder()));

        m_operatorController.a()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L1, ElevatorTarget.AlgaeScore);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL1);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetGroundIntakeAngle);
                }));

        m_operatorController.x()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L2);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL2_3);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetIntakeAngleL2);
                }));

        m_operatorController.b()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L3);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL2_3);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetIntakeAngleL3);
                }));

        m_operatorController.y()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L4, ElevatorTarget.AlgaeScore);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL4);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetScoreAngle);
                }));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
