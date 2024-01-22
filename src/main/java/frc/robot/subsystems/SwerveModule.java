// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.CtreUtils.configureCANCoder;
import static frc.robot.utils.CtreUtils.configureTalonFx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ROBOT;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.SWERVE.MODULE;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ModuleMap.MODULE_POSITION;
import frc.robot.visualizers.SwerveModuleVisualizer;
import java.io.File;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase implements AutoCloseable {
  private final ModuleMap.MODULE_POSITION m_modulePosition;
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_angleEncoder;
  private boolean invertDirection = false;

  private final double m_angleOffset;
  private final SwerveModulePosition m_internalState = new SwerveModulePosition();
  private Rotation2d m_lastHeadingR2d;
  private Pose2d m_pose;
  private boolean m_initSuccess = false;
  private SwerveModuleState m_desiredState = new SwerveModuleState();

  private final StatusSignal<Double> m_drivePosition;
  private final StatusSignal<Double> m_driveVelocity;
  private final StatusSignal<Double> m_turnPosition;
  private final StatusSignal<Double> m_turnVelocity;
  private final BaseStatusSignal[] m_signals = new BaseStatusSignal[4];

  private final VoltageOut m_voltageOut = new VoltageOut(0);
  private final DutyCycleOut driveMotorDutyControl = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0);
  private final PositionVoltage turnPositionControl = new PositionVoltage(0);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          MODULE.ksDriveVoltSecondsPerMeter,
          MODULE.kvDriveVoltSecondsSquaredPerMeter,
          MODULE.kaDriveVoltSecondsSquaredPerMeter);

  private SwerveModuleVisualizer m_moduleVisualizer;

  //  private DCMotorSim m_turnMotorSim =
  //      new DCMotorSim(MODULE.kTurnGearbox, MODULE.kTurnMotorGearRatio, 0.5);
  //  private DCMotorSim m_driveMotorSim =
  //      new DCMotorSim(
  //          //          LinearSystemId.createDCMotorSystem(0.134648227, 0.002802309),
  //          LinearSystemId.createDCMotorSystem(0.02, 0.001),
  //          MODULE.kDriveGearbox,
  //          MODULE.kDriveMotorGearRatio);
  private DCMotorSim m_turnMotorSim =
      new DCMotorSim(MODULE.kTurnGearbox, MODULE.kTurnMotorGearRatio, MODULE.kTurnInertia);
  private DCMotorSim m_driveMotorSim =
      new DCMotorSim(MODULE.kDriveGearbox, MODULE.kDriveMotorGearRatio, MODULE.kDriveInertia);

  public SwerveModule(
      MODULE_POSITION modulePosition,
      TalonFX turnMotor,
      TalonFX driveMotor,
      CANcoder angleEncoder,
      double angleOffset,
      boolean invertDirection) {
    m_modulePosition = modulePosition;
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = RobotBase.isReal() ? angleOffset : 0;

    if (RobotBase.isSimulation()) {
      m_angleEncoder.setPosition(0);
    }

    configureCANCoder(m_angleEncoder, CtreUtils.generateCanCoderConfig());
    var turnConfig = CtreUtils.generateTurnMotorConfig();
    var driveConfig = CtreUtils.generateTurnMotorConfig();
    if (invertDirection) {
      turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    configureTalonFx(m_turnMotor, turnConfig);
    configureTalonFx(m_driveMotor, driveConfig);

    m_turnPosition = m_turnMotor.getPosition().clone();
    m_turnVelocity = m_turnMotor.getVelocity().clone();
    m_drivePosition = m_driveMotor.getPosition().clone();
    m_driveVelocity = m_driveMotor.getVelocity().clone();
    m_signals[0] = m_drivePosition;
    m_signals[1] = m_driveVelocity;
    m_signals[2] = m_turnPosition;
    m_signals[3] = m_turnVelocity;

    setTurnAngle(0);

    m_lastHeadingR2d = getTurnHeadingR2d();
    m_moduleVisualizer = new SwerveModuleVisualizer(this.getName(), DRIVE.kMaxSpeedMetersPerSecond);

    initSmartDashboard();

    // To distinguish modules in CommandScheduler
    setName("SwerveModule_" + m_modulePosition.ordinal());

    SmartDashboard.putData(
        "SwerveModule2D_" + m_modulePosition.ordinal(), m_moduleVisualizer.getMechanism2d());
  }

  public boolean getInitSuccess() {
    return m_initSuccess;
  }

  public BaseStatusSignal[] getSignals() {
    return m_signals;
  }

  public MODULE_POSITION getModulePosition() {
    return m_modulePosition;
  }

  public Rotation2d getTurnEncoderAbsHeading() {
    m_angleEncoder.getAbsolutePosition().refresh();
    if (invertDirection)
      return Rotation2d.fromRotations(-m_angleEncoder.getAbsolutePosition().getValue());
    else return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
  }

  public void setTurnAngle(double angle) {
    var newAngle = getTurnEncoderAbsHeading().getDegrees() - m_angleOffset + angle;

    StatusCode turnMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      turnMotorStatus = m_turnMotor.setPosition(newAngle / 360.0);
      if (turnMotorStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }

    if (!turnMotorStatus.isOK()) {
      System.out.println(
          "Could not update Swerve Turn TalonFX Angle: "
              + m_turnMotor.getDeviceID()
              + ". Error code: "
              + turnMotorStatus);
    } else {
      System.out.printf(
          """
                      Updated Turn Motor %2d Angle:
                      Desired Angle: %.2f
                      Turn Motor Angle: %.2f
                      CANCoder Absolute Angle: %.2f
                      CANCoder Offset: %.2f\n""",
          m_turnMotor.getDeviceID(),
          angle,
          getTurnHeadingDeg(),
          getTurnEncoderAbsHeading().getDegrees(),
          m_angleOffset);
    }
  }

  public double getTurnHeadingDeg() {
    return getTurnHeadingR2d().getDegrees();
  }

  public Rotation2d getTurnHeadingR2d() {
    return getPosition(true).angle;
  }

  public double getDriveMps() {
    m_driveVelocity.refresh();
    return m_driveVelocity.getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public double getDriveMeters() {
    return getPosition(true).distanceMeters;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    m_desiredState = SwerveModuleState.optimize(desiredState, getTurnHeadingR2d());

    if (isOpenLoop) {
      double percentOutput = m_desiredState.speedMetersPerSecond / DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.setControl(driveMotorDutyControl.withOutput(percentOutput));
    } else {
      double velocityRPS =
          m_desiredState.speedMetersPerSecond / (MODULE.kWheelDiameterMeters * Math.PI);
      m_driveMotor.setControl(
          driveVelocityControl
              .withVelocity(velocityRPS)
              .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));
    }

    var heading =
        (Math.abs(m_desiredState.speedMetersPerSecond) <= (DRIVE.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastHeadingR2d
            : m_desiredState.angle; // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.setControl(turnPositionControl.withPosition(heading.getRotations()));
    m_lastHeadingR2d = heading;
  }

  public void initDriveSysid() {
    CtreUtils.configureTalonFx(m_driveMotor, new TalonFXConfiguration());
    CtreUtils.configureTalonFx(m_turnMotor, CtreUtils.generateTurnMotorConfig());
    setDriveBrake();
    BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        m_driveMotor.getPosition(),
        m_driveMotor.getVelocity(),
        m_driveMotor.getMotorVoltage());

    m_driveMotor.optimizeBusUtilization();
  }

  public void setDriveSysidVoltage(double volts) {
    m_driveMotor.setControl(m_voltageOut.withOutput(volts));
    m_turnMotor.setControl(turnPositionControl.withPosition(0));
  }

  public void initTurnSysid() {
    CtreUtils.configureTalonFx(m_turnMotor, new TalonFXConfiguration());

    BaseStatusSignal.setUpdateFrequencyForAll(
        250, m_turnMotor.getPosition(), m_turnMotor.getVelocity(), m_turnMotor.getMotorVoltage());

    m_turnMotor.optimizeBusUtilization();

    var signalLoggerDir = new File("/home/lvuser/logger/sysid/");
    if (!signalLoggerDir.exists()) {
      signalLoggerDir.mkdirs();
    }

    SignalLogger.setPath(signalLoggerDir.getAbsolutePath());
    System.out.println("Finished Initializing Turn Settings");
  }

  public void setTurnSysidVoltage(double volts) {
    m_turnMotor.setControl(m_voltageOut.withOutput(volts));
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(getDriveMps(), getTurnHeadingR2d());
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      m_drivePosition.refresh();
      m_driveVelocity.refresh();
      m_turnPosition.refresh();
      m_turnVelocity.refresh();
    }

    /* Now latency-compensate our signals */
    double drive_rot =
        BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
    double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(m_turnPosition, m_turnVelocity);

    /*
     * Back out the drive rotations based on angle rotations due to coupling between
     * azimuth and steer
     */
    //    drive_rot -= angle_rot * m_couplingRatioDriveRotorToCANcoder;
    drive_rot -= angle_rot * 3.5714285714285716;

    /* And push them into a SwerveModulePosition object to return */
    m_internalState.distanceMeters = drive_rot / (MODULE.kWheelDiameterMeters * Math.PI);
    /* Angle is already in terms of steer rotations */
    m_internalState.angle = Rotation2d.fromRotations(angle_rot);

    return m_internalState;
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getModulePose() {
    return m_pose;
  }

  public void setDriveBrake() {
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setDriveNeutral() {
    m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setTurnBrake() {
    m_turnMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setTurnCoast() {
    m_turnMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  private void initSmartDashboard() {
    setName("SwerveModule" + m_modulePosition.ordinal());
  }

  private void updateSmartDashboard() {}

  public void updateLog() {
    Logger.recordOutput(
        String.format("Swerve/Module %d/Encoder Absolute Position", m_modulePosition.ordinal()),
        getTurnEncoderAbsHeading().getDegrees());
    Logger.recordOutput(
        String.format("Swerve/Module %d/Turn Motor Position", m_modulePosition.ordinal()),
        getTurnHeadingDeg());
    Logger.recordOutput(
        String.format("Swerve/Module %d/Drive Motor Velocity", m_modulePosition.ordinal()),
        getDriveMps());

    // Debug
    //    Logger.recordOutput(
    //        String.format("Swerve/Module %d/Drive Motor Desired Velocity",
    // m_modulePosition.ordinal()),
    //        m_desiredState.speedMetersPerSecond);
    //    Logger.recordOutput(
    //        String.format("Swerve/Module %d/Drive Motor Setpoint", m_modulePosition.ordinal()),
    //        Double.valueOf(
    //            m_driveMotor.getAppliedControl().getControlInfo().getOrDefault("Velocity", "0")));
  }

  public void updateSimState(double deltaTime, double supplyVoltage) {
    TalonFXSimState simTurnMotor = m_turnMotor.getSimState();
    TalonFXSimState simDriveMotor = m_driveMotor.getSimState();
    CANcoderSimState simCanCoder = m_angleEncoder.getSimState();

    simTurnMotor.setSupplyVoltage(supplyVoltage);
    simDriveMotor.setSupplyVoltage(supplyVoltage);
    simCanCoder.setSupplyVoltage(supplyVoltage);

    m_turnMotorSim.setInputVoltage(
        addFriction(simTurnMotor.getMotorVoltage(), MODULE.kFrictionVoltage));
    m_driveMotorSim.setInputVoltage(
        addFriction(simDriveMotor.getMotorVoltage(), MODULE.kFrictionVoltage));

    m_turnMotorSim.update(deltaTime);
    m_driveMotorSim.update(deltaTime);

    simTurnMotor.setRawRotorPosition(
        m_turnMotorSim.getAngularPositionRotations() * MODULE.kTurnMotorGearRatio);
    simTurnMotor.setRotorVelocity(
        m_turnMotorSim.getAngularVelocityRPM() / 60.0 * MODULE.kTurnMotorGearRatio);

    /* CANcoders see the mechanism, so don't account for the steer gearing */
    simCanCoder.setRawPosition(m_turnMotorSim.getAngularPositionRotations());
    simCanCoder.setVelocity(m_turnMotorSim.getAngularVelocityRPM() / 60.0);

    simDriveMotor.setRawRotorPosition(
        m_driveMotorSim.getAngularPositionRotations() * MODULE.kDriveMotorGearRatio);
    simDriveMotor.setRotorVelocity(
        m_driveMotorSim.getAngularVelocityRPM() / 60.0 * MODULE.kDriveMotorGearRatio);
  }

  private double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }
    return motorVoltage;
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    if (!ROBOT.disableLogging) updateLog();

    if (!ROBOT.disableVisualization) m_moduleVisualizer.update(getCurrentState());
  }

  @Override
  public void simulationPeriodic() {
    //    RoboRioSim.setVInVoltage(
    //        BatterySim.calculateDefaultBatteryLoadedVoltage(
    //            m_turnMotorSim.getCurrentDrawAmps(), m_driveMotorSim.getCurrentDrawAmps()));
    //
    //    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12,
    // 12));
    //    m_driveMotorSim.setInputVoltage(
    //        MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));
    //
    //    double dt = RobotTime.getTimeDelta();
    //    m_turnMotorSim.update(dt);
    //    m_driveMotorSim.update(dt);
    //
    //    m_turnMotorSimState.setSupplyVoltage(
    //        RobotController.getBatteryVoltage()
    //            - m_turnMotorSim.getCurrentDrawAmps() * kMotorResistance);
    //    m_driveMotorSimState.setSupplyVoltage(
    //        RobotController.getBatteryVoltage()
    //            - m_driveMotorSim.getCurrentDrawAmps() * kMotorResistance);
    //
    //    var turnVelocityRps = m_turnMotorSim.getAngularVelocityRPM() / 60.0;
    //    var driveVelocityRps =
    //        m_driveMotorSim.getAngularVelocityRPM() * MODULE.kDriveMotorGearRatio / 60.0;
    //
    //    m_turnMotorSimState.setRawRotorPosition(
    //        m_turnMotorSim.getAngularPositionRotations() * MODULE.kTurnMotorGearRatio);
    //    m_turnMotorSimState.setRotorVelocity(turnVelocityRps * MODULE.kTurnMotorGearRatio);
    //    m_angleEncoderSimState.setRawPosition(m_turnMotorSim.getAngularPositionRotations());
    //    m_angleEncoderSimState.setVelocity(turnVelocityRps);
    //    m_driveMotorSimState.setRawRotorPosition(
    //        m_driveMotorSim.getAngularPositionRotations() * MODULE.kDriveMotorGearRatio);
    //    m_driveMotorSimState.setRotorVelocity(driveVelocityRps);
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_turnMotor.close();
    m_angleEncoder.close();
  }
}
