// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import frc.robot.Robot;

public class DriverCameras extends SubsystemBase {
    UsbCamera m_driverCamera;
    UsbCamera m_coralCamera;

    public DriverCameras() {
        if (Robot.isSimulation())
            return;

        m_driverCamera = CameraServer.startAutomaticCapture(0);
        m_coralCamera = CameraServer.startAutomaticCapture(1);

        m_driverCamera.setFPS(30);
        m_coralCamera.setFPS(30);
    }

    @Override
    public void periodic() {
    }
}
