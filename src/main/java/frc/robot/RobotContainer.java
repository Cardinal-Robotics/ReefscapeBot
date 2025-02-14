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

import frc.robot.Constants.AlgaeMechanismConstants;
import frc.robot.commands.ToggleableAlgaeIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriverCameras;
import frc.robot.commands.AlignAprilTag;

import com.pathplanner.lib.auto.AutoBuilder;

import swervelib.SwerveInputStream;

public class RobotContainer {
    // Misc.
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SendableChooser<Command> m_autoChooser;
    private static final SendableChooser<Boolean> m_displayDebugData = new SendableChooser<Boolean>();
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
    //
    // private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
    // private final ElevatorSubsystem m_elevatorSubsystem = new
    // ElevatorSubsystem();
    private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveDrive.getLibSwerveDrive());
    private final DriverCameras m_driverCameras = new DriverCameras();

    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // YAGSL Swerve input streams
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SwerveInputStream m_driveAngularVelocity = SwerveInputStream.of(m_swerveDrive.getLibSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> (m_driverController.getRightX() * -1)) // The
            // simulation
            // has
            // inverted
            // rotation.
            .deadband(OperatorConstants.kDeadband) // The joystick has to exceed the deadband for it to register. This
                                                   // way slight micro-movements doesn't suddenly move the robot.
            .scaleTranslation(0.8) // If the joystick goes to 100%, this scales it down to 80%.
            .allianceRelativeControl(true); // Field orientation flips to be on the your team's side.

    private final SwerveInputStream m_driveDirectAngle = m_driveAngularVelocity.copy()
            .withControllerHeadingAxis(
                    m_driverController::getRightX,
                    m_driverController::getRightY)
            .headingWhile(true);

    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Commands
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final Command m_driveFieldOrientedDirectAngle = m_swerveDrive.driveFieldOriented(m_driveDirectAngle);
    private final Command m_driveFieldOrientedAngularVelocity = m_swerveDrive
            .driveFieldOriented(m_driveAngularVelocity);
    private final Command m_resetGyro = Commands.runOnce(() -> m_swerveDrive.resetGyro(), m_swerveDrive);

    // private final ToggleableAlgaeIntake m_toggleableAlgaeIntake = new
    // ToggleableAlgaeIntake(m_algaeSubsystem);
    private final AlignAprilTag m_alignAprilTag = new AlignAprilTag(m_visionSubsystem, m_swerveDrive);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        configureBindings();

        m_swerveDrive.setDefaultCommand(m_driveFieldOrientedAngularVelocity);

        m_displayDebugData.addOption("Display", true);
        m_displayDebugData.addOption("Hide", false);
        SmartDashboard.putData("Display debug data?", m_displayDebugData);

        m_autoChooser = AutoBuilder.buildAutoChooser("AlexGreat");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    public static boolean shouldDisplayDebugData() {
        return m_displayDebugData.getSelected();
    }

    private void configureBindings() {
        // Driver controls
        m_driverController.y().onTrue(m_resetGyro);
        m_driverController.a().whileTrue(m_alignAprilTag);
        /*
         * m_driverController.b()
         * .onTrue(m_swerveDrive.driveToPose(DriverStation.getAlliance().get() ==
         * Alliance.Red
         * ? Constants.DriveConstants.kInitialRedRobotPose
         * : Constants.DriveConstants.kInitialBlueRobotPose));
         */

        /*
         * m_operatorController.x()
         * .onTrue(Commands.runOnce(
         * () -> m_elevatorSubsystem.setElevatorGoal(ElevatorConstants.
         * kElevatorPositionScoreAlgae),
         * m_elevatorSubsystem));
         * m_operatorController.povDown()
         * .onTrue(Commands.runOnce(
         * () ->
         * m_elevatorSubsystem.setElevatorGoal(ElevatorConstants.kElevatorPositionL1),
         * m_elevatorSubsystem));
         * m_operatorController.povUp()
         * .onTrue(Commands.runOnce(
         * () ->
         * m_elevatorSubsystem.setElevatorGoal(ElevatorConstants.kElevatorPositionL2),
         * m_elevatorSubsystem));
         * m_operatorController.povLeft()
         * .onTrue(Commands.runOnce(
         * () ->
         * m_elevatorSubsystem.setElevatorGoal(ElevatorConstants.kElevatorPositionL3),
         * m_elevatorSubsystem));
         * m_operatorController.povRight()
         * .onTrue(Commands.runOnce(
         * () ->
         * m_elevatorSubsystem.setElevatorGoal(ElevatorConstants.kElevatorPositionL4),
         * m_elevatorSubsystem));
         */

        // Operator controls
        // m_operatorController.b().onTrue(m_toggleableAlgaeIntake);
        // m_operatorController.leftTrigger().whileTrue(m_releaseAlgae);
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
