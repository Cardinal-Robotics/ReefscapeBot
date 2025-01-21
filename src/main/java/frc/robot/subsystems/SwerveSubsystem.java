// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDriveTest;
import swervelib.SwerveDrive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.DriveConstants;

import java.util.function.Supplier;

public class SwerveSubsystem extends SubsystemBase {
    StructPublisher<Pose2d> m_publisher = NetworkTableInstance.getDefault()
            .getStructTopic("YAGSL Pose", Pose2d.struct).publish();
    private SwerveDrive m_swerveDrive;

    private ChassisSpeeds maxVelocity = new ChassisSpeeds(0, 0, 0);

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // REMOVE OR LOW FOR COMP MAKES RUN SLOW
        try {
            m_swerveDrive = new SwerveParser(DriveConstants.kSwerveDirectory)
                    .createSwerveDrive(DriveConstants.kMaxSpeed);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        m_swerveDrive.resetOdometry(getInitialPose());
        m_swerveDrive.field.setRobotPose(getInitialPose());
        m_publisher.set(m_swerveDrive.getPose());

        setupPathPlanner();
    }

    @Override
    public void periodic() {
        this.m_publisher.set(m_swerveDrive.getPose());
    }

    private Pose2d getInitialPose() {
        // For robot simulation you will need to use the real DriverStation since the
        // alliance is obtained from DriverStation.
        var alliance = DriverStation.getAlliance();

        if (!alliance.isPresent())
            return DriveConstants.kInitialBlueRobotPose;
        if (alliance.get() == DriverStation.Alliance.Red)
            return DriveConstants.kInitialRedRobotPose;

        return DriveConstants.kInitialBlueRobotPose;
    }

    public void resetGyro() {
        this.m_swerveDrive.setGyro(Rotation3d.kZero);
    }

    public void lockInPlace() {
        m_swerveDrive.lockPose();
    }

    public void driveRelative(double x, double y, double rotation) {
        ChassisSpeeds velocity = new ChassisSpeeds(x, y, Units.degreesToRadians(rotation));
        m_swerveDrive.drive(velocity);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        m_swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            this.m_swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * Use PathPlanner to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                m_swerveDrive.getMaximumChassisVelocity(), 4.0,
                m_swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
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

            AutoBuilder.resetOdom(getInitialPose()).schedule();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}


    
    
        
    
