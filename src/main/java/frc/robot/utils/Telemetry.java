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

  private final SwerveModuleVisualizer[] m_moduleVisualizer = {
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.FRONT_LEFT.name(), m_maxSpeed),
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.FRONT_RIGHT.name(), m_maxSpeed),
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.BACK_LEFT.name(), m_maxSpeed),
    new SwerveModuleVisualizer(ModuleMap.MODULE_POSITION.BACK_RIGHT.name(), m_maxSpeed)
  };

  private final Pose2d[] m_swerveModulePoses = {
    new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d(),
  };
  private final Transform2d[] m_moduleTransforms = new Transform2d[4];
  private final double[] m_moduleAngles = new double[4];

  /** Construct a telemetry object */
  public Telemetry() {}

  public void registerFieldSim(FieldSim fieldSim) {
    m_fieldSim = fieldSim;
  }

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d m_lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  /* Accept the swerve drive state and telemeterize it to SmartDashboard */
  public void telemeterize(SwerveDriveState state) {
    Pose2d pose = state.Pose;

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
    m_lastPose = pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    try {
      Logger.recordOutput("Swerve/Pose", pose);
      Logger.recordOutput("Swerve/Speed", velocities.getNorm());
      Logger.recordOutput("Swerve/Velocity X", velocities.getX());
      Logger.recordOutput("Swerve/Velocity Y", velocities.getY());
      Logger.recordOutput("Swerve/Odometry Period", state.OdometryPeriod);
      Logger.recordOutput("Swerve/Module Targets", state.ModuleTargets);
      Logger.recordOutput("Swerve/Module States", state.ModuleStates);
      m_moduleAngles[0] = state.ModuleStates[0].angle.getDegrees();
      m_moduleAngles[1] = state.ModuleStates[1].angle.getDegrees();
      m_moduleAngles[2] = state.ModuleStates[2].angle.getDegrees();
      m_moduleAngles[3] = state.ModuleStates[3].angle.getDegrees();

      Logger.recordOutput("Swerve/Module Angles", m_moduleAngles);
    } catch (Exception e) {
      System.out.println("AdvantageKit could not update Odometry");
    }

    if (m_fieldSim != null) {
      for (ModuleMap.MODULE_POSITION i : ModuleMap.MODULE_POSITION.values()) {
        m_moduleVisualizer[i.ordinal()].update(state.ModuleStates[i.ordinal()]);
        m_moduleTransforms[i.ordinal()] =
            new Transform2d(
                SWERVE.DRIVE.kModuleTranslations.get(i), state.ModuleStates[i.ordinal()].angle);
        m_swerveModulePoses[i.ordinal()] = pose.transformBy(m_moduleTransforms[i.ordinal()]);
      }

      m_fieldSim.updateRobotPose(pose);
      m_fieldSim.updateSwervePoses(m_swerveModulePoses);
    }
  }
}
