// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.Constants.AlgaeMechanismConstants;
import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.SimulatedGame;
import frc.robot.subsystems.DriverCameras;
import frc.robot.commands.AlignAprilTag;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

import swervelib.SwerveInputStream;

public class RobotContainer {
    // Misc.
    // ---------------------------------------------------------------------------------------------------------------------------------------
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
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();

    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveDrive.getLibSwerveDrive());
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem(m_elevatorSubsystem, m_swerveDrive);
    private final CoralSubsystem m_coralSubsystem = new CoralSubsystem(m_elevatorSubsystem);
    private final LightSubsystem m_lightSubsystem = new LightSubsystem(m_elevatorSubsystem);
    // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final DriverCameras m_driverCameras = new DriverCameras();

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
                    () -> m_driverController.getRightX() * -1,
                    () -> m_driverController.getRightY() * -1)
            .headingWhile(true)
            .deadband(OperatorConstants.kDeadband) // The joystick has to exceed the deadband for it
                                                   // to register. This way slight micro-movements doesn't suddenly move
                                                   // the robot.
            .scaleTranslation(0.8) // If the joystick goes to 100%, this scales it down to 80%.
            .allianceRelativeControl(true); // Field orientation flips to be on the your team's side.
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Commands
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final Command m_driveFieldOriented = m_swerveDrive
            .driveFieldOriented(m_driveInputStream);
    private final Command m_resetGyro = Commands.runOnce(() -> m_swerveDrive.resetGyro(), m_swerveDrive);

    private final AlignAprilTag m_alignAprilTag = new AlignAprilTag(m_visionSubsystem, m_swerveDrive);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        registerNamedCommands();
        configureBindings();

        m_swerveDrive.setDefaultCommand(m_driveFieldOriented);

        m_autoChooser = AutoBuilder.buildAutoChooser("Leave");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        // m_coralsubsystem.setDefaultCommand(m_coralCommand);
        m_lightSubsystem.setDefaultCommand(
                DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                        ? m_lightSubsystem.setConstantColor(253, 11, 205)
                        : m_lightSubsystem.setConstantColor(0, 0, 255));
    }

    private void registerNamedCommands() {
        // AprilTag Alignment
        NamedCommands.registerCommand("AprilTagAlign",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive));

        // Elevator positions
        NamedCommands.registerCommand("ElevatorCoralIntake", Commands
                .runOnce(() -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.CoralIntake), m_elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorCoralL1",
                Commands.runOnce(() -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L1), m_elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorCoralL2",
                Commands.runOnce(() -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L2), m_elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorCoralL3",
                Commands.runOnce(() -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L3), m_elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorCoralL4",
                Commands.runOnce(() -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L4), m_elevatorSubsystem));

        // Coral mechanism
        NamedCommands.registerCommand("CoralTiltIntake", Commands
                .runOnce(() -> m_coralSubsystem.setTarget(CoralMechanismConstants.kIntakePosition), m_coralSubsystem)
                .until(() -> m_coralSubsystem.atTarget()));
        NamedCommands.registerCommand("CoralTiltIdle", Commands
                .runOnce(() -> m_coralSubsystem.setTarget(CoralMechanismConstants.kCoralStore), m_coralSubsystem)
                .until(() -> m_coralSubsystem.atTarget()));
        NamedCommands.registerCommand("CoralTiltL1", Commands
                .runOnce(() -> m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL1), m_coralSubsystem)
                .until(() -> m_coralSubsystem.atTarget()));
        NamedCommands.registerCommand("CoralTiltL2-L3", Commands
                .runOnce(() -> m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL2_3), m_coralSubsystem)
                .until(() -> m_coralSubsystem.atTarget()));
        NamedCommands.registerCommand("CoralTiltL4", Commands
                .runOnce(() -> m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL4), m_coralSubsystem)
                .until(() -> m_coralSubsystem.atTarget()));
        NamedCommands.registerCommand("CoralRelease", Commands
                .runOnce(() -> m_coralSubsystem.spinIntakeMotor(0.2), m_coralSubsystem)
                .until(() -> m_coralSubsystem.atTarget()));
        NamedCommands.registerCommand("CoralStopRelease", Commands
                .runOnce(() -> m_coralSubsystem.spinIntakeMotor(0), m_coralSubsystem)
                .until(() -> m_coralSubsystem.atTarget()));
    }

    private void configureBindings() {
        // Driver controls
        m_driverController.y().onTrue(m_resetGyro);
        m_driverController.a().whileTrue(m_alignAprilTag);

        m_driverController.povRight().onTrue(Commands.runOnce(() -> m_alignAprilTag.setOffsetPose(0.15, -0.5)));
        m_driverController.povLeft().onTrue(Commands.runOnce(() -> m_alignAprilTag.setOffsetPose(-0.2, -0.5)));
        m_driverController.povUp().onTrue(Commands.runOnce(() -> m_alignAprilTag.setOffsetPose(0, -0.5)));

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
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kCoralStore);
                }));

        m_operatorController.leftStick()
                .onTrue(Commands.runOnce(() -> {
                    interactionState = InteractionState.Coral;
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetDisabledAngle);
                }));

        // Coral Controls
        m_operatorController.rightBumper()
                .onTrue(m_coralSubsystem.setMotors(1))
                .onFalse(m_coralSubsystem.setMotors(0));
        m_operatorController.leftBumper()
                .onTrue(m_coralSubsystem.setMotors(-1))
                .onFalse(m_coralSubsystem.setMotors(0));

        // Algae Controls
        m_operatorController.rightTrigger()
                .onTrue(m_algaeSubsystem.setMotors(-1))
                .onFalse(m_algaeSubsystem.setMotors(0));
        m_operatorController.leftTrigger()
                .onTrue(m_algaeSubsystem.setMotors(1))
                .onFalse(m_algaeSubsystem.setMotors(0));

        // Elevator Positions
        m_operatorController.button(7)
                .onTrue(Commands.runOnce(() -> {
                    m_lightSubsystem.elevatorPattern();
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.CoralIntake);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kIntakePosition);
                }, m_elevatorSubsystem));
        m_operatorController.a()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L1, ElevatorTarget.AlgaeScore);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL1);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetGroundIntakeAngle);
                }, m_elevatorSubsystem));
        m_operatorController.x()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L2);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL2_3);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetIntakeAngle);
                }, m_elevatorSubsystem));
        m_operatorController.b()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L3);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL2_3);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetIntakeAngle);
                }, m_elevatorSubsystem));
        m_operatorController.y()
                .onTrue(Commands.runOnce(() -> {
                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L4, ElevatorTarget.AlgaeScore);
                    m_coralSubsystem.setTarget(CoralMechanismConstants.kTargetAngleL4);
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetScoreAngle);
                }, m_elevatorSubsystem));

    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
