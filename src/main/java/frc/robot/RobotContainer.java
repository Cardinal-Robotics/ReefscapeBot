// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    //
    // Controllers
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final XboxController m_driverController = new XboxController(
            OperatorConstants.kDriverControllerPort);
    private final XboxController m_operatorController = new XboxController(
            OperatorConstants.kOperatorControllerPort);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Subsystems
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();

    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // YAGSL Swerve input streams
    // ---------------------------------------------------------------------------------------------------------------------------------------

    // Gets how fast it should move.
    private final SwerveInputStream m_driveAngularVelocity = SwerveInputStream.of(m_swerveDrive.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(m_driverController::getRightX)
            .deadband(OperatorConstants.kDeadband) // The joystick has to exceed the deadband for it to register. This
                                                   // way slight micro-movements doesn't suddenly move the robot.
            .scaleTranslation(0.8) // If the joystick goes to 100%, this scales it down to 80%.
            .allianceRelativeControl(true); // Field orientation flips to be on the your team's side.

    // Gets which angle to turn to.
    private final SwerveInputStream m_driveDirectAngle = m_driveAngularVelocity.copy()
            .withControllerHeadingAxis(
                    m_driverController::getRightX,
                    m_driverController::getRightY)
            .headingWhile(true);

    // YAGSL requires special logic for robot simulation. Taken from:
    // https://github.com/BroncBotz3481/YAGSL-Example/blob/de932a45b41409442ffcc99832597906ebd34eb4/src/main/java/frc/robot/RobotContainer.java#L92-L110
    private final SwerveInputStream m_driveAngularVelocitySim = SwerveInputStream.of(m_swerveDrive.getSwerveDrive(),
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX())
            .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
            .deadband(OperatorConstants.kDeadband)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    private final SwerveInputStream m_driveDirectAngleSim = m_driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(() -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
            .headingWhile(true);

    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    //
    // Commands
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final Command m_driveFieldOrientedDirectAngle = m_swerveDrive.driveFieldOriented(m_driveDirectAngle);
    private final Command m_driveFieldOrientedAngularVelocity = m_swerveDrive
            .driveFieldOriented(m_driveAngularVelocity);

    // YAGSL requires special logic for robot simulation.
    private final Command m_driveFieldOrientedDirectAngleSim = m_swerveDrive.driveFieldOriented(m_driveDirectAngleSim);
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    public RobotContainer() {
        configureBindings();

        // This is called a "Ternary operator"; When you do something along the lines of
        // variable = (condition) ? expressionTrue : expressionFalse; variable =
        // expressionTrue when the condition is true, and expressionFalse when the
        // condition is false. For example:
        // displayMessage = (2 + 2) == 22 ? "Math ain't adding up" : "Correct!";
        // Then displayMessage will be "Correct!" because 4 doesn't equal 22.
        m_swerveDrive.setDefaultCommand(
                RobotBase.isSimulation() ? m_driveFieldOrientedDirectAngleSim : m_driveFieldOrientedDirectAngle);
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
