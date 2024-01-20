// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.SWERVE.DRIVE.kMaxSpeedMetersPerSecond;
import static frc.robot.constants.SWERVE.DRIVE.kSwerveKinematics;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BASE;
import frc.robot.constants.CAN;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ModuleMap.MODULE_POSITION;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

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

  private final Pigeon2 m_pigeon = new Pigeon2(CAN.pigeon, "rio");
  private Pigeon2SimState m_pigeonSim;

  private double m_rollOffset;

  private boolean m_limitJoystickInput = false;

  private final SwerveDrivePoseEstimator m_odometry;

  private MechanismLigament2d m_swerveChassis2d;

  @SuppressWarnings("CanBeFinal")
  private boolean m_simOverride = false; // DO NOT MAKE FINAL. WILL BREAK UNIT TESTS

  private double m_simYaw;
  private double m_simRoll;
  private DoublePublisher pitchPub, rollPub, yawPub, odometryXPub, odometryYPub, odometryYawPub;

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
    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDrivePoseEstimator(
            DRIVE.kSwerveKinematics,
            getHeadingRotation2d(),
            getSwerveDriveModulePositionsArray(),
            new Pose2d());

    m_turnController.enableContinuousInput(-Math.PI, Math.PI);

    if (RobotBase.isReal()) {
      resetModulesToAbsolute();
    } else {
      m_pigeonSim = m_pigeon.getSimState();
    }

    initSmartDashboard();
    setTurnMotorCoast();
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
      boolean isFieldRelative,
      boolean isOpenLoop) {
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

    Map<MODULE_POSITION, SwerveModuleState> moduleStates =
        ModuleMap.of(DRIVE.kSwerveKinematics.toSwerveModuleStates(newChassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), m_currentMaxVelocity);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
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

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_currentMaxVelocity);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    var states = DRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(states, false);
  }

  public void setOdometry(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      m_pigeon.getSimState().setRawYaw(pose.getRotation().getDegrees());
    } else m_pigeon.setYaw(pose.getRotation().getDegrees());
    m_odometry.resetPosition(getHeadingRotation2d(), getSwerveDriveModulePositionsArray(), pose);

    for (var position : MODULE_POSITION.values()) {
      var transform =
          new Transform2d(DRIVE.kModuleTranslations.get(position), Rotation2d.fromDegrees(0));
      var modulePose = pose.plus(transform);
      getSwerveModule(position).setTurnAngle(pose.getRotation().getDegrees());
      getSwerveModule(position).setModulePose(modulePose);
    }
  }

  public void setRollOffset() {
    m_rollOffset = -m_pigeon.getRoll().getValue(); // -2.63
  }

  public double getRollOffsetDegrees() {
    return m_rollOffset;
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
    return m_odometry.getEstimatedPosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(
        m_swerveModules.get(MODULE_POSITION.FRONT_LEFT).getState(),
        m_swerveModules.get(MODULE_POSITION.FRONT_RIGHT).getState(),
        m_swerveModules.get(MODULE_POSITION.BACK_LEFT).getState(),
        m_swerveModules.get(MODULE_POSITION.BACK_RIGHT).getState());
  }

  public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
    Map<MODULE_POSITION, SwerveModuleState> moduleStates =
        ModuleMap.of(DRIVE.kSwerveKinematics.toSwerveModuleStates(targetSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), m_currentMaxVelocity);

    setSwerveModuleStatesAuto(moduleStates.values().toArray(new SwerveModuleState[0]));
  }

  public SwerveModule getSwerveModule(MODULE_POSITION modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public Map<MODULE_POSITION, SwerveModuleState> getModuleStates() {
    Map<MODULE_POSITION, SwerveModuleState> map = new HashMap<>();
    for (MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getState());
    return map;
  }

  public Map<MODULE_POSITION, SwerveModulePosition> getModulePositions() {
    Map<MODULE_POSITION, SwerveModulePosition> map = new HashMap<>();
    for (MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getPosition());
    return map;
  }

  public SwerveModulePosition[] getSwerveDriveModulePositionsArray() {
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

  public SwerveDrivePoseEstimator getOdometry() {
    return m_odometry;
  }

  public void resetGyro() {
    //    if (DriverStation.isFMSAttached() && Controls.getAllianceColor() ==
    // DriverStation.Alliance.Red)
    //      m_pigeon.setYaw(180);
    //    else
    m_pigeon.setYaw(0);
  }

  public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getSwerveDriveModulePositionsArray());

    if (!BASE.disableVisualization)
      for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
        Transform2d moduleTransform =
            new Transform2d(
                DRIVE.kModuleTranslations.get(module.getModulePosition()),
                module.getTurnHeadingR2d());
        module.setModulePose(getPoseMeters().transformBy(moduleTransform));
      }
  }

  private void initSmartDashboard() {
    setName("SwerveDrive");
    SmartDashboard.putData(this);
  }

  private void updateSmartDashboard() {}

  private void updateLog() {}

  @Override
  public void periodic() {
    if (DriverStation.isEnabled() && useHeadingTarget) {
      calculateRotationSpeed();
    }

    updateOdometry();
    updateSmartDashboard();
    if (!BASE.disableLogging) updateLog();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed =
        DRIVE.kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    m_simYaw += chassisSpeed.omegaRadiansPerSecond * RobotTime.getTimeDelta();

    m_pigeonSim.setRawYaw(-Units.radiansToDegrees(m_simYaw));
    }

    @Override
  public void close() throws Exception {
    if (m_swerveChassis2d != null) m_swerveChassis2d.close();
    for (var module : ModuleMap.orderedValuesList(m_swerveModules)) module.close();
  }
}
