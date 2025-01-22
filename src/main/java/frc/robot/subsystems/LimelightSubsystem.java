// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_table;

    public LimelightSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("limelight"); // gets 8 values from limelight
    }

    public boolean hasTargets() {
        return m_table.getEntry("tv").getInteger(0) == 1;
    }

    public double getTX() {
        return m_table.getEntry("tx").getDouble(Double.NaN);
    }

    public double getTY() {
        return m_table.getEntry("ty").getDouble(Double.NaN);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
