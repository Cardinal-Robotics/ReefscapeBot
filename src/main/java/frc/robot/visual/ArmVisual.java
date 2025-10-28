// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visual;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmVisual extends SubsystemBase {
    private final SingleJointedArmSim m_armSim;
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;

    public ArmVisual(SingleJointedArmSim armSim) {
        this.m_armSim = armSim;

        mech = new Mechanism2d(3, 3);
        root = mech.getRoot("Pivot", 1.5, 1.5); // Pivot in center
        arm = root.append(
                new MechanismLigament2d("Arm", 0.5, 0));

        SmartDashboard.putData("Arm Visualization", mech);
    }

    @Override
    public void simulationPeriodic() {
        arm.setAngle(Math.toDegrees(m_armSim.getAngleRads()));
    }
}
