// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
    /** Creates a new LimelightSubsystem. */
    NetworkTable table;
    SwerveSubsystem m_swerveSubsystem;

    public LimelightSubsystem(SwerveSubsystem subsystem) {
        m_swerveSubsystem = subsystem;
    }

    public void align() {
        table = NetworkTableInstance.getDefault().getTable("limelight"); // gets 8 values from limelight
        double x = table.getEntry("tx").getDouble(0);
        double y = table.getEntry("ty").getDouble(0);
        SmartDashboard.putNumber("Tag x-offset", x); // logs values to make sure we get them
        SmartDashboard.putNumber("Tag y-offset", y);

        // this aligns first with x then goes with y once x aligned
        if (x > 2) { // moves robot right
            m_swerveSubsystem.driveRelative(0, 0.8, 0);
        } else if (x < -2) { // left
            m_swerveSubsystem.driveRelative(0, -0.8, 0);
        }

        if (x < 2 && x > -2) { // once x aligned, y
            if (y < 11) { // forward
                m_swerveSubsystem.driveRelative(-0.8, 0, 0);
            } else if (y > 15) { // back
                m_swerveSubsystem.driveRelative(0.8, 0, 0);

            }
        }

        // .245,

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
