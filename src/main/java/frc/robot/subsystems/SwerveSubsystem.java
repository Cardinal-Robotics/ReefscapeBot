// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive m_swerveDrive;

    private ChassisSpeeds maxVelocity = new ChassisSpeeds(0, 0, 0);

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // REMOVE OR LOW FOR COMP MAKES RUN SLOW
        try {
            m_swerveDrive = new SwerveParser(DriveConstants.kSwerveDirectory)
                    .createSwerveDrive(DriveConstants.kMaxSpeed);

        } catch (Exception e) {
            SmartDashboard.putString("SwerveSubsystem", "Failed to create YAGSL Swerve Drive");
            throw new RuntimeException(e);
        }

        setupPathPlanner();
    }

    public void updatePIDF(PIDFConfig drivePIDF, PIDFConfig anglePIDF) {
        SwerveModule[] modules = m_swerveDrive.getModules();
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            module.setDrivePIDF(drivePIDF);
            module.setAnglePIDF(anglePIDF);
        }
    }

    public void lockInPlace() {
        m_swerveDrive.lockPose();
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        m_swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            this.m_swerveDrive.driveFieldOriented(velocity.get());
            if (maxVelocity.vxMetersPerSecond < this.getLibSwerveDrive()
                    .getFieldVelocity().vxMetersPerSecond) {
                maxVelocity = this.getLibSwerveDrive().getFieldVelocity();
                SmartDashboard.putNumber("max x velocity", maxVelocity.vxMetersPerSecond);
            }
        });
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive, 12, false),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive),
                3.0, 5.0, 3.0);
    }

    /**
     * @return Returns the YAGSL Swerve Drive instance
     */
    public SwerveDrive getLibSwerveDrive() {
        return this.m_swerveDrive;
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    m_swerveDrive::getPose,
                    m_swerveDrive::resetOdometry,
                    m_swerveDrive::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        m_swerveDrive.drive(
                                speedsRobotRelative,
                                m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                moduleFeedForwards.linearForces());
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(DriveConstants.kPTrans, DriveConstants.kITrans, DriveConstants.kDTrans), // Translation
                                                                                                                      // PID
                            new PIDConstants(DriveConstants.kPAngular, DriveConstants.kIAngular,
                                    DriveConstants.kDAngular)), // Rotation PID
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
