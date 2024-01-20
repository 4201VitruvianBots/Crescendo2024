package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("Limelight2");

  public Vision() {}

  public boolean isCameraConnected() {
    return camera.isConnected();
  }

  public boolean isAprilTagDetected() {
    if (camera.isConnected()) {
      var result = camera.getLatestResult();
      return result.hasTargets();
    } else {
      return false;
    }
  }

  public boolean is3DPoseEstimationAvailable() {
    if (camera.isConnected()) {
      var result = camera.getLatestResult();
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        return PhotonUtils.is3DPoseEstimationAvailable(target);
      }
    }
    return false;
  }

  public void updateLog() {
    Logger.recordOutput("vision/isCameraConnected", isCameraConnected());
    Logger.recordOutput("vision/isAprilTagDetected", isAprilTagDetected());
    Logger.recordOutput("vision/is3DPoseEstimationAvailable", is3DPoseEstimationAvailable());

    if (camera.isConnected()) {
      var result = camera.getLatestResult();
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        Logger.recordOutput("vision/targetX", target.getX());
        Logger.recordOutput("vision/targetY", target.getY());
        Logger.recordOutput("vision/targetArea", target.getArea());
        Logger.recordOutput("vision/targetDistance", target.getDistance());
        Logger.recordOutput("vision/targetRotation", target.getRotation());
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLog();
    smartDashboard();
  }
}
