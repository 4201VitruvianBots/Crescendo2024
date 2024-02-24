package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ROBOT;
import frc.robot.constants.VISION;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private CommandSwerveDrivetrain m_swerveDriveTrain;

  private final NetworkTable NoteDetectionLimelight =
      NetworkTableInstance.getDefault().getTable("limelight");

  public static final PhotonCamera aprilTagLimelightCameraA = new PhotonCamera("LimelightA");
  PhotonPoseEstimator limelightPhotonPoseEstimatorA =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          aprilTagLimelightCameraA,
          VISION.robotToAprilTagLimelightCameraA);

  public static final PhotonCamera aprilTagLimelightCameraB = new PhotonCamera("LimelightB");
  PhotonPoseEstimator limelightPhotonPoseEstimatorB =
      new PhotonPoseEstimator(
          VISION.aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          aprilTagLimelightCameraB,
          VISION.robotToAprilTagLimelightCameraB);

  private VisionSystemSim visionSim;
  private PhotonCameraSim aprilTagLimelightCameraASim;
  private PhotonCameraSim aprilTagLimelightCameraBSim;

  public Vision() {
    if (RobotBase.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(VISION.aprilTagFieldLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      aprilTagLimelightCameraASim = new PhotonCameraSim(aprilTagLimelightCameraA, cameraProp);
      aprilTagLimelightCameraBSim = new PhotonCameraSim(aprilTagLimelightCameraB, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(aprilTagLimelightCameraASim, VISION.robotToAprilTagLimelightCameraA);
      visionSim.addCamera(aprilTagLimelightCameraBSim, VISION.robotToAprilTagLimelightCameraB);

      aprilTagLimelightCameraASim.enableDrawWireframe(false);
      aprilTagLimelightCameraBSim.enableDrawWireframe(false);
    }
  }

  public void registerSwerveDrive(CommandSwerveDrivetrain swerveDriveTrain) {
    m_swerveDriveTrain = swerveDriveTrain;
  }

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
    Logger.recordOutput("vision/NoteDetectionLimelight - isNoteDetected", hasGamePieceTarget());
    Logger.recordOutput(
        "vision/NoteDetectionLimelight - robotToGamePieceRotation", getRobotToGamePieceDegrees());

    Logger.recordOutput(
        "vision/LimelightA - isCameraConnected", isCameraConnected(aprilTagLimelightCameraA));
    Logger.recordOutput(
        "vision/LimelightA - isAprilTagDetected", isAprilTagDetected(aprilTagLimelightCameraA));
    Logger.recordOutput("vision/limelightA - targets", getTargets(aprilTagLimelightCameraA));

    Logger.recordOutput(
        "vision/LimelightB - isCameraConnected", isCameraConnected(aprilTagLimelightCameraB));
    Logger.recordOutput(
        "vision/LimelightB - isAprilTagDetected", isAprilTagDetected(aprilTagLimelightCameraB));
    Logger.recordOutput("vision/limelightB - targets", getTargets(aprilTagLimelightCameraB));
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

  @Override
  public void simulationPeriodic() {
    if (m_swerveDriveTrain != null) {
      visionSim.update(m_swerveDriveTrain.getState().Pose);
      visionSim.getDebugField().setRobotPose(m_swerveDriveTrain.getState().Pose);
    }
  }
}
