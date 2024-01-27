package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VISION;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase {
  
  PhotonCamera camera = new PhotonCamera("Limelight2");
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          camera,
          VISION.robotToCam
  );

  private Pose2d estimatedPose;

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
      return String.join(" ", targets.stream().map(PhotonTrackedTarget::toString).toList());
    } else {
      return "No targets";
    }
  }

  public PhotonTrackedTarget getTarget() {
    if (camera.isConnected()) {
      var result = camera.getLatestResult();
      List<PhotonTrackedTarget> targets = result.getTargets();
      if (!targets.isEmpty()) {
        return targets.get(0);
      } else   {
        return null;
      }
    } else {
      return null;
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    var result = photonPoseEstimator.update();
    result.ifPresent(pose -> estimatedPose = pose.estimatedPose.toPose2d());
    return result;
  }

  public Pose3d getEstimatedFieldPose() {
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();

    var aprilTagDetection = VISION.aprilTagFieldLayout.getTagPose(target.getFiducialId());
    if(aprilTagDetection.isPresent()) {
      return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagDetection.get(), VISION.robotToCam);
    } else {
      return null;
    }
  }

  private void updateLog() {
    Logger.recordOutput("vision/isCameraConnected", isCameraConnected());
    Logger.recordOutput("vision/isAprilTagDetected", isAprilTagDetected());
    Logger.recordOutput("vision/getTargets", getTargets());
    Logger.recordOutput("vision/estimatedPose", estimatedPose);
    Logger.recordOutput("vision/estimated3DPose", getEstimatedFieldPose());
  }

  private void smartDashboard() {
    // Implement the smartDashboard method here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLog();
    smartDashboard();
  }
}
