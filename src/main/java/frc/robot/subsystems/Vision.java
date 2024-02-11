package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ROBOT;
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

  private CommandSwerveDrivetrain m_swerveDriveTrain;

  public Vision(CommandSwerveDrivetrain swerveDrivetrain) {
    m_swerveDriveTrain = swerveDrivetrain;
  }

  private FieldSim m_fieldSim;

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  public static PhotonCamera aprilTagLimelightCameraA = new PhotonCamera("AprilTagLimelightCameraA");
  PhotonPoseEstimator limelightPhotonPoseEstimatorA =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          aprilTagLimelightCameraA,
          VISION.robotToCam);

  public static PhotonCamera aprilTagLimelightCameraB = new PhotonCamera("AprilTagLimelightCameraB");
  PhotonPoseEstimator limelightPhotonPoseEstimatorB =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          aprilTagLimelightCameraB,
          VISION.robotToCam);

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator photonEstimator) {
    return photonEstimator.update();
  }

  public boolean isCameraConnected(PhotonCamera camera) {
    return camera.isConnected();
  }

  public boolean isAprilTagDetected(PhotonCamera camera) {
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public String getTargets(PhotonCamera camera) {
    var result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    return String.join(" ", targets.stream().map(PhotonTrackedTarget::toString).toList());
  }

  public boolean hasGamePieceTarget() {
    return false;
  }
  // TODO implement acutally (Bengi)//


  // TODO implement Acutally (Bengi)//

  public Rotation2d getRobotToGamePieceRotation() {
    return new Rotation2d();
  }

  private void updateLog() {
    Logger.recordOutput("vision/isAprilTagLimelightAConnected", isCameraConnected(aprilTagLimelightCameraA));
    Logger.recordOutput("vision/isAprilTagLimelightBConnected", isCameraConnected(aprilTagLimelightCameraB));

    if (isCameraConnected(aprilTagLimelightCameraA)) {
      Logger.recordOutput("vision/isAprilTagDetectedLimelightA", isAprilTagDetected(aprilTagLimelightCameraA));
    }
      if (isAprilTagDetected(aprilTagLimelightCameraA)) {
        Logger.recordOutput("vision/aprilTagLimelightATargetsDetected", getTargets(aprilTagLimelightCameraA));
      }
    

    if (isCameraConnected(aprilTagLimelightCameraB)) {
      Logger.recordOutput("vision/isAprilTagDetectedLimelightB", isAprilTagDetected(aprilTagLimelightCameraB));
    }
      if (isAprilTagDetected(aprilTagLimelightCameraB)) {
        Logger.recordOutput("vision/aprilTagLimelightBTargetsDetected", getTargets(aprilTagLimelightCameraB));
      }
    
  }

  private void updateSmartDashboard() {
    // Implement the smartDashboard method here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!ROBOT.disableLogging) updateLog();
    updateSmartDashboard();
  }
}
