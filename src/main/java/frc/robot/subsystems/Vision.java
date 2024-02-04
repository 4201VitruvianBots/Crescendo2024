package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VISION;
import frc.robot.simulation.FieldSim;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private FieldSim m_fieldSim;

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  PhotonCamera camera = new PhotonCamera("Limelight2");
  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          camera,
          VISION.robotToCam);

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return photonPoseEstimator.update();
  }

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
      return String.join(" ", targets.stream().map(PhotonTrackedTarget::toString).toList());
    } else {
      return "No targets";
    }
  }

  private void updateLog() {
    Logger.recordOutput("vision/isCameraConnected", isCameraConnected());

    if (isCameraConnected()) {
      Logger.recordOutput("vision/isAprilTagDetected", isAprilTagDetected());
      Logger.recordOutput("vision/getTargets", getTargets());
    }
  }

  private void smartDashboard() {
    // Implement the smartDashboard method here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLog();
    smartDashboard();
    if (m_fieldSim != null) {
      var pose = getEstimatedGlobalPose();
      if (pose.isPresent()) {
        m_fieldSim.updateVisionPose(pose.get().estimatedPose.toPose2d());
      }
    }
  }
}
