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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.ElevatorConstants.ElevatorTarget;
import frc.robot.Constants.AlgaeMechanismConstants;
import frc.robot.Constants.CoralMechanismConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

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

    // Controllers
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    // Subsystems
    private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();


    //
    // YAGSL Swerve input streams
    // ---------------------------------------------------------------------------------------------------------------------------------------
    private final SwerveInputStream m_driveInputStream = SwerveInputStream.of(m_swerveDrive.getLibSwerveDrive(),
            () -> m_driverController.getLeftY() * -1
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? -1 : 1),
            () -> m_driverController.getLeftX() * -1
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? -1 : 1))
            .withControllerHeadingAxis( // Maps joystick rotation to rotation on field. So if the joystick goes bottom
                                        // right, the robot rotates to the bottom red from the perspective of your
                                        // alliance
                    () -> m_driverController.getRightX()
                            * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? -1 : 1)
                            * (SmartDashboard.getBoolean("Invert Rotation", false) ? -1 : 1),
                    () -> m_driverController.getRightY()
                            * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? -1 : 1)
                            * (SmartDashboard.getBoolean("Invert Rotation", false) ? -1 : 1))
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
    // ---------------------------------------------------------------------------------------------------------------------------------------
    //

    public RobotContainer() {
        SmartDashboard.putBoolean("Invert Translation", false);
        SmartDashboard.putBoolean("Invert Rotation", false);

        DriverStation.silenceJoystickConnectionWarning(true);

        m_swerveDrive.setDefaultCommand(m_driveFieldOriented);

        m_driverController.y().whileTrue(m_resetGyro);

        m_autoChooser = AutoBuilder.buildAutoChooser("Leave");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        m_autoChooser.onChange((Command selectedCommand) -> {
            String commandName = selectedCommand.getName();
            try {
                List<PathPlannerPath> trajectory = PathPlannerAuto.getPathGroupFromAutoFile(commandName);
                List<Pose2d> poses = new ArrayList<>();

                for (PathPlannerPath path : trajectory) {

                    if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red))
                        path = path.flipPath();

                    poses.addAll(path.getPathPoses());
                }

                m_swerveDrive.getLibSwerveDrive().field.getObject("trajectory").setPoses(poses);
            } catch (Exception exception) {

            }
        });


    }
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
