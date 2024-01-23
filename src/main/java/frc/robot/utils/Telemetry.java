package frc.robot.utils;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.SWERVE;
import frc.robot.simulation.FieldSim;
import frc.robot.visualizers.SwerveModuleVisualizer;
import org.littletonrobotics.junction.Logger;

public class Telemetry {
  private FieldSim m_fieldSim;
  private final double m_maxSpeed = SWERVE.DRIVE.kMaxSpeedMetersPerSecond;

  private SwerveModuleVisualizer[] m_moduleVisualizer = {
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.FRONT_LEFT.name(), m_maxSpeed),
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.FRONT_RIGHT.name(), m_maxSpeed),
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.BACK_LEFT.name(), m_maxSpeed),
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.BACK_RIGHT.name(), m_maxSpeed)
  };

  private Pose2d[] m_swerveModulePoses = {
    new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d(),
  };

  /** Construct a telemetry object */
  public Telemetry() {}

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d m_lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    var pose = state.Pose;
    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
    m_lastPose = pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    Logger.recordOutput("Swerve/Pose", pose);
    Logger.recordOutput("Swerve/Speed", velocities.getNorm());
    Logger.recordOutput("Swerve/Velocity X", velocities.getX());
    Logger.recordOutput("Swerve/Velocity Y", velocities.getY());
    Logger.recordOutput("Swerve/Odometry Period", state.OdometryPeriod);

    /* Telemeterize the module's states */
    for (ModuleMap.MODULE_POSITION i : ModuleMap.MODULE_POSITION.values()) {
      m_moduleVisualizer[i.ordinal()].update(state.ModuleStates[i.ordinal()]);
      var moduleTransform =
          new Transform2d(
              SWERVE.DRIVE.kModuleTranslations.get(i), state.ModuleStates[i.ordinal()].angle);
      m_swerveModulePoses[i.ordinal()] = pose.transformBy(moduleTransform);
    }

    if (m_fieldSim != null) {
      m_fieldSim.updateRobotPose(pose);
      m_fieldSim.updateSwervePoses(m_swerveModulePoses);
    }
  }
}
