// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive m_libSwerveDrive;

    public SwerveSubsystem() {
        try {
            m_libSwerveDrive = new SwerveParser(DriveConstants.kSwerveDirectory)
                    .createSwerveDrive(DriveConstants.kMaxSpeed);
        } catch (Exception e) {
            SmartDashboard.putString("SwerveSubsystem", "Failed to create YAGSL Swerve Drive");
            throw new RuntimeException(e);
        }

        setupPathPlanner();
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        m_libSwerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            this.m_libSwerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * @return Returns the YAGSL Swerve Drive instance
     */
    public SwerveDrive getLibSwerveDrive() {
        return this.m_libSwerveDrive;
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

            final boolean enableFeedforward = true;
            AutoBuilder.configure(
                    m_libSwerveDrive::getPose,
                    m_libSwerveDrive::resetOdometry,
                    m_libSwerveDrive::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            m_libSwerveDrive.drive(
                                    speedsRobotRelative,
                                    m_libSwerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            m_libSwerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
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
