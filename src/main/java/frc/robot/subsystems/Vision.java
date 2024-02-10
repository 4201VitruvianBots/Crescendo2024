package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VISION;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

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

  // private Pose2d estimatedPose = new Pose2d();

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

  // public PhotonTrackedTarget getTarget() {
  //   if (camera.isConnected()) {
  //     var result = camera.getLatestResult();
  //     List<PhotonTrackedTarget> targets = result.getTargets();
  //     if (!targets.isEmpty()) {
  //       return targets.get(0);
  //     } else {
  //       return null;
  //     }
  //   } else {
  //     return null;
  //   }
  // }

  // public Pose3d getEstimatedFieldPose() {
  //   var result = camera.getLatestResult();

  //   if (result.hasTargets()) {
  //     PhotonTrackedTarget target = result.getBestTarget();

  //     if (target != null) {
  //       var aprilTagDetection = VISION.aprilTagFieldLayout.getTagPose(target.getFiducialId());
  //       if (aprilTagDetection.isPresent()) {
  //         return PhotonUtils.estimateFieldToRobotAprilTag(
  //             target.getBestCameraToTarget(), aprilTagDetection.get(), VISION.robotToCam);
  //       }
  //     }
  //   }

  //   return new Pose3d();
  // }

  // TODO implement acutally//
  public boolean hasGamePieceTarget() {
    return false;
  }

  // ToDO implement Acutally//
  public Rotation2d getRobotToGamePieceRotation() {
    return new Rotation2d();
  }

  private void updateLog() {
    Logger.recordOutput("vision/isCameraConnected", isCameraConnected());

    if (isCameraConnected()) {
      Logger.recordOutput("vision/isAprilTagDetected", isAprilTagDetected());
      Logger.recordOutput("vision/getTargets", getTargets());
      // Logger.recordOutput("vision/estimatedPose", estimatedPose);
      // Logger.recordOutput("vision/estimated3DPose", getEstimatedFieldPose());
      // Logger.recordOutput("vision/estimatedGlobalPose", getEstimatedGlobalPose());
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
  }
}
