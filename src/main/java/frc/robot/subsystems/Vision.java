package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  private FieldSim m_fieldSim;

  public Vision() {}

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  NetworkTable NoteDetectionLimelight = NetworkTableInstance.getDefault().getTable("limelight");

  public static PhotonCamera aprilTagLimelightCameraA =
      new PhotonCamera("AprilTagLimelightCameraA");
  PhotonPoseEstimator limelightPhotonPoseEstimatorA =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          aprilTagLimelightCameraA,
          VISION.robotToAprilTagLimelightCameraA);

  public static PhotonCamera aprilTagLimelightCameraB =
      new PhotonCamera("AprilTagLimelightCameraB");
  PhotonPoseEstimator limelightPhotonPoseEstimatorB =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          aprilTagLimelightCameraB,
          VISION.robotToAprilTagLimelightCameraB);

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

  public Boolean hasGamePieceTarget() {
    NetworkTableEntry tv = NoteDetectionLimelight.getEntry("tv");
    return tv.getDouble(0.0) == 1;
  }

  public double getRobotToGamePieceDegrees() {
    double degreesRotation = 0.0;
    if (hasGamePieceTarget()) {
      NetworkTableEntry tx = NoteDetectionLimelight.getEntry("tx");
      degreesRotation = tx.getDouble(0.0);
    }
    return degreesRotation;
  }

  public Rotation2d getRobotToGamePieceRotation() {
    return Rotation2d.fromDegrees(getRobotToGamePieceDegrees());
  }

  private void updateLog() {
    Logger.recordOutput("vision/isNoteDetected", hasGamePieceTarget());
    Logger.recordOutput("vision/robotToGamePieceRotation", getRobotToGamePieceDegrees());

    Logger.recordOutput(
        "vision/isAprilTagLimelightAConnected", isCameraConnected(aprilTagLimelightCameraA));
    Logger.recordOutput(
        "vision/isAprilTagLimelightBConnected", isCameraConnected(aprilTagLimelightCameraB));

    if (isCameraConnected(aprilTagLimelightCameraA)) {
      Logger.recordOutput(
          "vision/isAprilTagDetectedLimelightA", isAprilTagDetected(aprilTagLimelightCameraA));
    }
    if (isAprilTagDetected(aprilTagLimelightCameraA)) {
      Logger.recordOutput(
          "vision/aprilTagLimelightATargetsDetected", getTargets(aprilTagLimelightCameraA));
    }

    if (isCameraConnected(aprilTagLimelightCameraB)) {
      Logger.recordOutput(
          "vision/isAprilTagDetectedLimelightB", isAprilTagDetected(aprilTagLimelightCameraB));
    }
    if (isAprilTagDetected(aprilTagLimelightCameraB)) {
      Logger.recordOutput(
          "vision/aprilTagLimelightBTargetsDetected", getTargets(aprilTagLimelightCameraB));
    }
  }

  private void updateSmartDashboard() {
    // Implement the smartDashboard method here
  }

  @Override
  public void periodic() {
    if (m_swerveDriveTrain != null) {
      final var globalPoseA = getEstimatedGlobalPose(limelightPhotonPoseEstimatorA);
      globalPoseA.ifPresent(
          estimatedRobotPose ->
              m_swerveDriveTrain.addVisionMeasurement(
                  estimatedRobotPose.estimatedPose.toPose2d(),
                  estimatedRobotPose.timestampSeconds));

      final var globalPoseB = getEstimatedGlobalPose(limelightPhotonPoseEstimatorB);
      globalPoseB.ifPresent(
          estimatedRobotPose ->
              m_swerveDriveTrain.addVisionMeasurement(
                  estimatedRobotPose.estimatedPose.toPose2d(),
                  estimatedRobotPose.timestampSeconds));
    }
    // This method will be called once per scheduler run
    if (!ROBOT.disableLogging) updateLog();
    updateSmartDashboard();
  }
}
