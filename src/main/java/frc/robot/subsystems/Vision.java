package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
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

  public String getTargets() {
    if (camera.isConnected()) {
      var result = camera.getLatestResult();
      List<PhotonTrackedTarget> targets = result.getTargets();
      String targetString = "";
      for (PhotonTrackedTarget target : targets) {
        targetString += target + " ";
      }
      return targetString;
    } else {
      return "No targets";
    }
  }

  public PhotonTrackedTarget getTarget() {
    if (camera.isConnected()) {
      var result = camera.getLatestResult();
      List<PhotonTrackedTarget> targets = result.getTargets();
      if (targets.size() > 0) {
        return targets.get(0);
      } else   {
        return null;
      }
    } else {
      return null;
    }
  }

  public void updateLog() {
    Logger.recordOutput("vision/isCameraConnected", isCameraConnected());
    Logger.recordOutput("vision/isAprilTagDetected", isAprilTagDetected());
    Logger.recordOutput("vision/getTargets", getTargets());
  }

  public void smartDashboard() {
    // Implement the smartDashboard method here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLog();
    smartDashboard();
  }
}
