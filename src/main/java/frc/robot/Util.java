// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

public class Util {
    public static SparkMax getMotorFromSim(SparkMaxSim simulatedMotor) {
        try {
            Class<?> simulatedSparkClass = SparkMaxSim.class;
            Field privateField = simulatedSparkClass.getDeclaredField("m_spark");
            privateField.setAccessible(true);

            return (SparkMax) privateField.get(simulatedMotor);
        } catch (Exception error) {
            error.printStackTrace();
            throw new RuntimeException(
                    "\nFailed to extract motor. Please use 'new CoralSimVisual(simMotor, motor);' instead.");
        }
    }
}
