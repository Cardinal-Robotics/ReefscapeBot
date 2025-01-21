// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LimelightCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.UpdatePIDF;
import swervelib.SwerveInputStream;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
    // Misc.
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SendableChooser<Command> m_autoChooser;
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
    private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();
    private final LimelightSubsystem m_limelightsubsystem = new LimelightSubsystem(m_swerveDrive);

    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // YAGSL Swerve input streams
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SwerveInputStream m_driveAngularVelocity = SwerveInputStream.of(m_swerveDrive.getLibSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(m_driverController::getRightX)
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
    private final Command m_resetGyro = m_swerveDrive.resetGyro();
    private final LimelightCommand m_limelight = new LimelightCommand(m_limelightsubsystem);

    // private final UpdatePIDF m_updatePIDF = new UpdatePIDF(m_swerveDrive);

    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    public RobotContainer() {
        configureBindings();
        m_swerveDrive.setDefaultCommand(m_driveFieldOrientedAngularVelocity);

        m_autoChooser = AutoBuilder.buildAutoChooser("AlexGreat");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    private void configureBindings() {
        m_driverController.y().onTrue(m_resetGyro);
        m_driverController.a().whileTrue(m_limelight);
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

}

    