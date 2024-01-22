// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.SWERVE.DRIVE.kMaxSpeedMetersPerSecond;
import static frc.robot.constants.SWERVE.DRIVE.kSwerveKinematics;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ModuleMap.MODULE_POSITION;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class SwerveDrive extends SubsystemBase implements AutoCloseable {

  private final HashMap<MODULE_POSITION, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              MODULE_POSITION.FRONT_LEFT,
                  new SwerveModule(
                      MODULE_POSITION.FRONT_LEFT,
                      new TalonFX(CAN.frontLeftTurnMotor),
                      new TalonFX(CAN.frontLeftDriveMotor),
                      new CANcoder(CAN.frontLeftCanCoder),
                      DRIVE.frontLeftCANCoderOffset,
                      true),
              MODULE_POSITION.FRONT_RIGHT,
                  new SwerveModule(
                      MODULE_POSITION.FRONT_RIGHT,
                      new TalonFX(CAN.frontRightTurnMotor),
                      new TalonFX(CAN.frontRightDriveMotor),
                      new CANcoder(CAN.frontRightCanCoder),
                      DRIVE.frontRightCANCoderOffset,
                      true),
              MODULE_POSITION.BACK_LEFT,
                  new SwerveModule(
                      MODULE_POSITION.BACK_LEFT,
                      new TalonFX(CAN.backLeftTurnMotor),
                      new TalonFX(CAN.backLeftDriveMotor),
                      new CANcoder(CAN.backLeftCanCoder),
                      DRIVE.backLeftCANCoderOffset,
                      true),
              MODULE_POSITION.BACK_RIGHT,
                  new SwerveModule(
                      MODULE_POSITION.BACK_RIGHT,
                      new TalonFX(CAN.backRightTurnMotor),
                      new TalonFX(CAN.backRightDriveMotor),
                      new CANcoder(CAN.backRightCanCoder),
                      DRIVE.backRightCANCoderOffset,
                      true)));

  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];
  private SwerveModuleState[] m_desiredStates = {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
  };
  private final Pigeon2 m_pigeon = new Pigeon2(CAN.pigeon, "rio");
  private Pigeon2SimState m_pigeonSim;
  private final StatusSignal<Double> m_yaw;
  private final StatusSignal<Double> m_angularVelocity;

  private boolean m_limitJoystickInput = false;

  private final SwerveDrivePoseEstimator m_odometry;
  private final OdometryThread m_odometryThread;
  private final ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
  private final SwerveDrivetrain.SwerveDriveState m_cachedState =
      new SwerveDrivetrain.SwerveDriveState();

  private MechanismLigament2d m_swerveChassis2d;

  @SuppressWarnings("CanBeFinal")
  private boolean m_simOverride = false; // DO NOT MAKE FINAL. WILL BREAK UNIT TESTS

  private Notifier m_simNotifier;
  private Rotation2d m_simYaw = new Rotation2d();

  private boolean useHeadingTarget = false;
  private double m_desiredHeadingRadians;

  private final PIDController m_xController = new PIDController(DRIVE.kP_X, DRIVE.kI_X, DRIVE.kD_X);
  private final PIDController m_yController = new PIDController(DRIVE.kP_Y, DRIVE.kI_Y, DRIVE.kD_Y);
  private final PIDController m_turnController =
      new PIDController(DRIVE.kP_Theta, DRIVE.kI_Theta, DRIVE.kD_Theta);

  private double m_rotationOutput;

  ChassisSpeeds chassisSpeeds;
  private final double m_maxVelocity = kMaxSpeedMetersPerSecond;
  private final double m_limitedVelocity = DRIVE.kLimitedSpeedMetersPerSecond;
  private double m_currentMaxVelocity = m_maxVelocity;

  public SwerveDrive() {
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
    m_yaw = m_pigeon.getYaw().clone();
    m_angularVelocity = m_pigeon.getAngularVelocityZWorld().clone();
    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDrivePoseEstimator(
            DRIVE.kSwerveKinematics,
            getHeadingRotation2d(),
            getSwerveModulesPositionsArray(),
            new Pose2d());

    m_turnController.enableContinuousInput(-Math.PI, Math.PI);

    if (RobotBase.isReal()) {
      resetModulesToAbsolute();
    } else {
      m_pigeonSim = m_pigeon.getSimState();
      startSimThread();
    }

    initSmartDashboard();
    setTurnMotorCoast();

    m_odometryThread = new OdometryThread();
    m_odometryThread.start();
  }

  private void resetModulesToAbsolute() {
    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) module.setTurnAngle(0);
  }

  public void setJoystickLimit(boolean limit) {
    m_limitJoystickInput = limit;
  }

  public void drive(
      double throttle,
      double strafe,
      double rotation,
      boolean isFieldRelative) {
    if (m_limitJoystickInput) {
      throttle *= m_limitedVelocity;
      strafe *= m_limitedVelocity;
      rotation *= DRIVE.kLimitedRotationRadiansPerSecond;
    } else {
      throttle *= m_currentMaxVelocity;
      strafe *= m_currentMaxVelocity;
      rotation *= DRIVE.kMaxRotationRadiansPerSecond;
    }

    /** Setting field vs Robot Relative */
    if (useHeadingTarget) {
      rotation = m_rotationOutput;
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getHeadingRotation2d());
    } else {
      chassisSpeeds =
          isFieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  throttle, strafe, rotation, getHeadingRotation2d())
              : new ChassisSpeeds(throttle, strafe, rotation);
    }
    var newChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, RobotTime.getTimeDelta());

    var states = DRIVE.kSwerveKinematics.toSwerveModuleStates(newChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_currentMaxVelocity);

    setSwerveModuleStates(states);
  }

  /** Set robot heading to a clear target */
  public void setHeadingTargetRadians(double radians) {
    m_desiredHeadingRadians = MathUtil.inputModulus(radians, -Math.PI, Math.PI);
  }

  public void calculateRotationSpeed() {
    if (Math.abs(getHeadingRotation2d().getRadians() - m_desiredHeadingRadians)
        > Units.degreesToRadians(1))
      m_rotationOutput =
          m_turnController.calculate(getHeadingRotation2d().getRadians(), m_desiredHeadingRadians);
    else m_rotationOutput = 0;
  }

  /*
   * ability to let head of swerve drive face the target
   */
  public void enableHeadingTarget(boolean enable) {
    useHeadingTarget = enable;
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_currentMaxVelocity);

    try {
      m_stateLock.writeLock().lock();

      m_desiredStates = states;
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    var states = DRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(states);
  }

  public void setOdometry(Pose2d pose) {
    try {
      m_stateLock.writeLock().lock();

      m_odometry.resetPosition(
          Rotation2d.fromDegrees(m_yaw.getValue()), getSwerveModulesPositionsArray(), pose);
      /* We need to update our cached pose immediately so that race conditions don't happen */
      m_cachedState.Pose = pose;
    } finally {
      m_stateLock.writeLock().unlock();
    }

    for (var position : MODULE_POSITION.values()) {
      var transform =
          new Transform2d(DRIVE.kModuleTranslations.get(position), Rotation2d.fromDegrees(0));
      var modulePose = pose.plus(transform);
      getSwerveModule(position).setTurnAngle(pose.getRotation().getDegrees());
      getSwerveModule(position).setModulePose(modulePose);
    }
  }

  public double getPitchDegrees() {
    return m_pigeon.getPitch().getValue();
  }

  public double getRollDegrees() {
    return m_pigeon.getRoll().getValue();
  }

  public double getHeadingDegrees() {
    return m_pigeon.getYaw().getValue();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getPoseMeters() {
    try {
      m_stateLock.readLock().lock();

      return m_cachedState.Pose;
    } finally {
      m_stateLock.readLock().unlock();
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(
        m_swerveModules.get(MODULE_POSITION.FRONT_LEFT).getCurrentState(),
        m_swerveModules.get(MODULE_POSITION.FRONT_RIGHT).getCurrentState(),
        m_swerveModules.get(MODULE_POSITION.BACK_LEFT).getCurrentState(),
        m_swerveModules.get(MODULE_POSITION.BACK_RIGHT).getCurrentState());
  }

  public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
    Map<MODULE_POSITION, SwerveModuleState> moduleStates =
        ModuleMap.of(DRIVE.kSwerveKinematics.toSwerveModuleStates(targetSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), m_currentMaxVelocity);

    setSwerveModuleStates(moduleStates.values().toArray(new SwerveModuleState[0]));
  }

  public SwerveModule getSwerveModule(MODULE_POSITION modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public Map<MODULE_POSITION, SwerveModuleState> getModuleStates() {
    Map<MODULE_POSITION, SwerveModuleState> map = new HashMap<>();
    for (MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getCurrentState());
    return map;
  }

  public Map<MODULE_POSITION, SwerveModulePosition> getModulePositions() {
    Map<MODULE_POSITION, SwerveModulePosition> map = new HashMap<>();
    for (MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getPosition(true));
    return map;
  }

  public SwerveModulePosition[] getSwerveModulesPositionsArray() {
    return ModuleMap.orderedValues(getModulePositions(), new SwerveModulePosition[0]);
  }

  public Map<MODULE_POSITION, Pose2d> getModulePoses() {
    Map<MODULE_POSITION, Pose2d> map = new HashMap<>();
    for (MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getModulePose());
    return map;
  }

  public boolean getModuleInitStatus() {
    for (MODULE_POSITION i : m_swerveModules.keySet()) {
      if (!m_swerveModules.get(i).getInitSuccess()) {
        return false;
      }
    }
    return true;
  }

  public PIDController getXPidController() {
    return m_xController;
  }

  public PIDController getYPidController() {
    return m_yController;
  }

  public PIDController getThetaPidController() {
    return m_turnController;
  }

  public void setTurnMotorCoast() {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setTurnCoast();
    }
  }

  public void setTurnBrake() {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setTurnBrake();
    }
  }

  public void setMaxVelocity(double mps) {
    m_currentMaxVelocity = mps;
  }

  public void initDriveSysid() {
    for (SwerveModule module : m_swerveModules.values()) {
      module.initDriveSysid();
    }

    var signalLoggerDir = new File("/home/lvuser/logger/sysid/");
    if (!signalLoggerDir.exists()) {
      var result = signalLoggerDir.mkdirs();
      System.out.println("mkdirs() result: " + result);
    }

    SignalLogger.setPath(signalLoggerDir.getAbsolutePath());
    System.out.println("Finished Initializing Drive Settings");
  }

  public void resetGyro() {
    //    if (DriverStation.isFMSAttached() && Controls.getAllianceColor() ==
    // DriverStation.Alliance.Red)
    //      m_pigeon.setYaw(180);
    //    else
    m_pigeon.setYaw(0);
  }

  public void updateOdometry() {
//    m_odometry.update(getHeadingRotation2d(), getSwerveModulesPositionsArray());

    if (!ROBOT.disableVisualization)
      for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
        Transform2d moduleTransform =
            new Transform2d(
                DRIVE.kModuleTranslations.get(module.getModulePosition()),
                module.getTurnHeadingR2d());
        module.setModulePose(getPoseMeters().transformBy(moduleTransform));
      }
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    try {
      m_stateLock.writeLock().lock();
      m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  private void initSmartDashboard() {
    setName("SwerveDrive");
    SmartDashboard.putData(this);
  }

  private void updateSmartDashboard() {}

  private void updateLog() {}

  /* Perform swerve module updates in a separate thread to minimize latency */
  public class OdometryThread {
    protected static final int START_THREAD_PRIORITY =
        1; // Testing shows 1 (minimum realtime) is sufficient for tighter
    // odometry loops.
    // If the odometry period is far away from the desired frequency,
    // increasing this may help

    protected final Thread m_thread;
    protected volatile boolean m_running = false;

    protected final BaseStatusSignal[] m_allSignals;

    protected final MedianFilter peakRemover = new MedianFilter(3);
    protected final LinearFilter lowPass = LinearFilter.movingAverage(50);
    protected double lastTime = 0;
    protected double currentTime = 0;
    protected double averageLoopTime = 0;
    protected int SuccessfulDaqs = 0;
    protected int FailedDaqs = 0;

    protected int lastThreadPriority = START_THREAD_PRIORITY;
    protected volatile int threadPriorityToSet = START_THREAD_PRIORITY;

    protected final boolean IsOnCANFD = false;
    protected final double UpdateFrequency;

    public OdometryThread() {
      m_thread = new Thread(this::run);
      /* Mark this thread as a "daemon" (background) thread
       * so it doesn't hold up program shutdown */
      m_thread.setDaemon(true);

      UpdateFrequency = IsOnCANFD ? 250 : 100;

      /* 4 signals for each module + 2 for Pigeon2 */
      m_allSignals = new BaseStatusSignal[(MODULE_POSITION.values().length * 4) + 2];
      for (MODULE_POSITION i : m_swerveModules.keySet()) {
        var signals = m_swerveModules.get(i).getSignals();
        m_allSignals[(i.ordinal() * 4) + 0] = signals[0];
        m_allSignals[(i.ordinal() * 4) + 1] = signals[1];
        m_allSignals[(i.ordinal() * 4) + 2] = signals[2];
        m_allSignals[(i.ordinal() * 4) + 3] = signals[3];
      }
      m_allSignals[m_allSignals.length - 2] = m_yaw;
      m_allSignals[m_allSignals.length - 1] = m_angularVelocity;
    }

    /** Starts the odometry thread. */
    public void start() {
      m_running = true;
      m_thread.start();
    }

    /** Stops the odometry thread. */
    public void stop() {
      stop(0);
    }

    /**
     * Stops the odometry thread with a timeout.
     *
     * @param millis The time to wait in milliseconds
     */
    public void stop(long millis) {
      m_running = false;
      try {
        m_thread.join(millis);
      } catch (final InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    public void run() {
      /* Make sure all signals update at the correct update frequency */
      BaseStatusSignal.setUpdateFrequencyForAll(UpdateFrequency, m_allSignals);
      Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

      /* Run as fast as possible, our signals will control the timing */
      while (m_running) {
        /* Synchronously wait for all signals in drivetrain */
        /* Wait up to twice the period of the update frequency */
        StatusCode status;
        if (IsOnCANFD) {
          status = BaseStatusSignal.waitForAll(2.0 / UpdateFrequency, m_allSignals);
        } else {
          /* Wait for the signals to update */
          Timer.delay(1.0 / UpdateFrequency);
          status = BaseStatusSignal.refreshAll(m_allSignals);
        }

        try {
          m_stateLock.writeLock().lock();

          lastTime = currentTime;
          currentTime = RobotTime.getTime();
          /* We don't care about the peaks, as they correspond to GC events, and we want the period generally low passed */
          averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

          /* Get status of first element */
          if (status.isOK()) {
            SuccessfulDaqs++;
          } else {
            FailedDaqs++;
          }

          /* Now update odometry */
          /* Keep track of the change in azimuth rotations */
          for (MODULE_POSITION i : m_swerveModules.keySet()) {
            m_modulePositions[i.ordinal()] = m_swerveModules.get(i).getPosition(false);
          }
          double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(m_yaw, m_angularVelocity);

          /* Keep track of previous and current pose to account for the carpet vector */
          m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);

          /* And now that we've got the new odometry, update the controls */
          //          m_requestParameters.currentPose = m_odometry.getEstimatedPosition()
          //                  .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset));
          //          m_requestParameters.kinematics = kSwerveKinematics;
          //          m_requestParameters.swervePositions = DRIVE.kModuleTranslations;
          //          m_requestParameters.timestamp = currentTime;
          //          m_requestParameters.updatePeriod = 1.0 / UpdateFrequency;
          //
          //          m_requestToApply.apply(m_requestParameters, Modules);
          for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
            module.setDesiredState(m_desiredStates[module.getModulePosition().ordinal()], false);
          }

          /* Update our cached state with the newly updated data */
          m_cachedState.FailedDaqs = FailedDaqs;
          m_cachedState.SuccessfulDaqs = SuccessfulDaqs;
          m_cachedState.Pose = m_odometry.getEstimatedPosition();
          m_cachedState.OdometryPeriod = averageLoopTime;

          if (m_cachedState.ModuleStates == null) {
            m_cachedState.ModuleStates = new SwerveModuleState[m_modulePositions.length];
          }
          if (m_cachedState.ModuleTargets == null) {
            m_cachedState.ModuleTargets = new SwerveModuleState[m_modulePositions.length];
          }
          for (MODULE_POSITION i : m_swerveModules.keySet()) {
            m_cachedState.ModuleStates[i.ordinal()] = m_swerveModules.get(i).getCurrentState();
            m_cachedState.ModuleTargets[i.ordinal()] = m_swerveModules.get(i).getDesiredState();
          }

          // TODO: Change this to AdvantageKit
          //          if (m_telemetryFunction != null) {
          //            /* Log our state */
          //            m_telemetryFunction.accept(m_cachedState);
          //          }
        } finally {
          m_stateLock.writeLock().unlock();
        }

        /**
         * This is inherently synchronous, since lastThreadPriority is only written here and
         * threadPriorityToSet is only read here
         */
        if (threadPriorityToSet != lastThreadPriority) {
          Threads.setCurrentThreadPriority(true, threadPriorityToSet);
          lastThreadPriority = threadPriorityToSet;
        }
      }
    }

    public boolean odometryIsValid() {
      return SuccessfulDaqs > 2; // Wait at least 3 daqs before saying the odometry is valid
    }

    /**
     * Sets the DAQ thread priority to a real time priority under the specified priority level
     *
     * @param priority Priority level to set the DAQ thread to. This is a value between 0 and 99,
     *     with 99 indicating higher priority and 0 indicating lower priority.
     */
    public void setThreadPriority(int priority) {
      threadPriorityToSet = priority;
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled() && useHeadingTarget) {
      calculateRotationSpeed();
    }

//    updateOdometry();
    updateSmartDashboard();
    if (!ROBOT.disableLogging) updateLog();
  }

  @Override
  public void simulationPeriodic() {
    //    var twist = kSwerveKinematics.toTwist2d(getSwerveModulesPositionsArray());
    //
    //    m_simYaw = m_simYaw.plus(Rotation2d.fromRadians(twist.dtheta));
    //
    //    m_pigeonSim.setRawYaw(-m_simYaw.getDegrees());
  }

  private void startSimThread() {
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              double deltaTime = RobotTime.getTimeDelta();

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(0.005);
  }

  private void updateSimState(double deltaTime, double supplyVoltage) {
    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      module.updateSimState(deltaTime, supplyVoltage);
    }

    // Update Chassis Heading
    ChassisSpeeds chassisSpeed =
        kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
    m_simYaw =
        m_simYaw.plus(Rotation2d.fromRadians(chassisSpeed.omegaRadiansPerSecond * deltaTime));
    m_pigeonSim.setRawYaw(m_simYaw.getDegrees());
  }

  @Override
  public void close() throws Exception {
    if (m_swerveChassis2d != null) m_swerveChassis2d.close();
    for (var module : ModuleMap.orderedValuesList(m_swerveModules)) module.close();
  }
}
