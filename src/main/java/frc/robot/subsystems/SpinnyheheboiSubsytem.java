// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpinnyheheboiSubsytem extends SubsystemBase {
    PWMVictorSPX victor = new PWMVictorSPX(9);

    /** Creates a new SpinnyheheboiSubsytem. */
    public SpinnyheheboiSubsytem() {
    }

    public void spin() {
        victor.setVoltage(10);
    }

    public void stop() {
        victor.setVoltage(0);
    }

    @Override
    public void periodic() {
        spin();
        // This method will be called once per scheduler run
    }
}
