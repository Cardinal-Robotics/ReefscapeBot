// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMechanism extends SubsystemBase {
    // private ArmVisual visual = new ArmVisual(m_armSim);

    public ArmMechanism() {
        SmartDashboard.putNumber("Target Angle", 0);
    }

    @Override
    public void periodic() {
        double angle = SmartDashboard.getNumber("Target Angle", 0);
    }

    @Override
    public void simulationPeriodic() {

    }
}
