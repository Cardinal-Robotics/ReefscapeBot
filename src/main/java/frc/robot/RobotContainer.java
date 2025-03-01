// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.commands.AlignAprilTag.TagPositions;
import frc.robot.Constants.AlgaeMechanismConstants;
import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.commands.ToggleableAlgaeIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.SimulatedGame;
import frc.robot.subsystems.DriverCameras;
import frc.robot.commands.AlignAprilTag;
import frc.robot.commands.LEDCommand;

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
    public static CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Subsystems
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();

    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveDrive.getLibSwerveDrive());
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
    private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
    private final DriverCameras m_driverCameras = new DriverCameras();
    // private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
    private final SimulatedGame m_gameSim = new SimulatedGame(m_elevatorSubsystem, m_algaeSubsystem);

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

    private final ToggleableAlgaeIntake m_toggleableAlgaeIntake = new ToggleableAlgaeIntake(m_elevatorSubsystem,
            m_algaeSubsystem);
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
        // m_ledSubsystem.setDefaultCommand(m_LEDCommand);
    }

    private void registerNamedCommands() {
        // AprilTag Alignment
        NamedCommands.registerCommand("AprilTagAlignTop",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, TagPositions.TOP));
        NamedCommands.registerCommand("AprilTagAlignTopRight",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, TagPositions.TOP_RIGHT));
        NamedCommands.registerCommand("AprilTagAlignTopLeft",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, TagPositions.TOP_LEFT));
        NamedCommands.registerCommand("AprilTagAlignBottom",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, TagPositions.BOTTOM));
        NamedCommands.registerCommand("AprilTagAlignBottomLeft",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, TagPositions.BOTTOM_LEFT));
        NamedCommands.registerCommand("AprilTagAlignBottomRight",
                new AlignAprilTag(m_visionSubsystem, m_swerveDrive, TagPositions.BOTTOM_RIGHT));

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
        /*
         * m_driverController.x()
         * .toggleOnTrue(Commands.runOnce(() ->
         * m_swerveDrive.setDefaultCommand(m_driveFieldOriented)));
         */

        // Climber controls
        m_driverController.leftStick()
                .onTrue(Commands.runOnce(() -> m_climberSubsystem.setGoal(ClimberConstants.kCrushingFrame),
                        m_climberSubsystem));
        m_driverController.rightStick()
                .onTrue(Commands.runOnce(() -> m_climberSubsystem.setGoal(ClimberConstants.kCrushingCage),
                        m_climberSubsystem));

        // Target AprilTag positions
        m_driverController.povUp().debounce(0.25)
                .onTrue(Commands.runOnce(() -> m_alignAprilTag.setTagPosition(TagPositions.TOP)));
        m_driverController.povUpRight()
                .onTrue(Commands.runOnce(() -> m_alignAprilTag.setTagPosition(TagPositions.TOP_RIGHT)));
        m_driverController.povUpLeft()
                .onTrue(Commands.runOnce(() -> m_alignAprilTag.setTagPosition(TagPositions.TOP_LEFT)));
        m_driverController.povDown().debounce(0.25)
                .onTrue(Commands.runOnce(() -> m_alignAprilTag.setTagPosition(TagPositions.BOTTOM)));
        m_driverController.povDownRight()
                .onTrue(Commands.runOnce(() -> m_alignAprilTag.setTagPosition(TagPositions.BOTTOM_RIGHT)));
        m_driverController.povDownLeft()
                .onTrue(Commands.runOnce(() -> m_alignAprilTag.setTagPosition(TagPositions.BOTTOM_LEFT)));

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
                .onTrue(Commands.runOnce(() -> interactionState = InteractionState.Algae));

        m_operatorController.leftStick()
                .onTrue(Commands.runOnce(() -> {
                    interactionState = InteractionState.Coral;
                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetDisabledAngle);
                }));

        // Coral Controls
        m_operatorController.rightBumper()
                .onTrue(m_coralSubsystem.setMotors(0.2))
                .onFalse(m_coralSubsystem.setMotors(0));
        m_operatorController.leftBumper()
                .onTrue(m_coralSubsystem.setMotors(-0.2))
                .onFalse(m_coralSubsystem.setMotors(0));

        // Algae Controls
        m_operatorController.rightTrigger()
                .onTrue(m_toggleableAlgaeIntake);

        // Elevator Positions
        m_operatorController.button(7)
                .onTrue(Commands.runOnce(
                        () -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.CoralIntake),
                        m_elevatorSubsystem));
        m_operatorController.button(8)
                .onTrue(Commands.runOnce(
                        () -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.AlgaeScore),
                        m_elevatorSubsystem));
        m_operatorController.a()
                .onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L1),
                        m_elevatorSubsystem));
        m_operatorController.x()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L2);
                                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetIntakeAngle);
                                },
                                m_elevatorSubsystem));
        m_operatorController.b()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L3);
                                    m_algaeSubsystem.setTiltTarget(AlgaeMechanismConstants.kTargetIntakeAngle);
                                },
                                m_elevatorSubsystem));
        m_operatorController.y()
                .onTrue(Commands.runOnce(
                        () -> m_elevatorSubsystem.setElevatorGoal(ElevatorTarget.L4),
                        m_elevatorSubsystem));

    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
