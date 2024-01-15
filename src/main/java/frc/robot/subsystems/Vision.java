// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("Limelight2");

  public Vision() {}

  public boolean isAprilTagDetected() {
    if(camera.isConnected()) {
    var result = camera.getLatestResult();
    return result.hasTargets();
    } else {
      return false;
    }
  }

  public void updateLog() {
    Logger.recordOutput("vision/isCameraConnected", camera.isConnected());

    Logger.recordOutput("vision/isAprilTagDetected", isAprilTagDetected());
  }

  @Override
  public void periodic() {
    updateLog();
  }
}
