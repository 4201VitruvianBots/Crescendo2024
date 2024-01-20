// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.simulation.SimConstants.kMotorResistance;
import static frc.robot.utils.CtreUtils.configureCANCoder;
import static frc.robot.utils.CtreUtils.configureTalonFx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
  private Rotation2d m_lastHeadingR2d;
  private Pose2d m_pose;
  private boolean m_initSuccess = false;
  private SwerveModuleState m_desiredState = new SwerveModuleState();
  private final VoltageOut m_voltageOut = new VoltageOut(0);
  private final DutyCycleOut driveMotorDutyControl = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0);
  private final PositionVoltage turnPositionControl = new PositionVoltage(0);

  private final SimpleMotorFeedforward m_turnFF =
      new SimpleMotorFeedforward(
          MODULE.ksTurnVoltsRotation,
          MODULE.kvTurnVoltSecondsPerRotation,
          MODULE.kaTurnVoltSecondsSquaredPerRotation);

  private final SimpleMotorFeedforward m_driveFF =
      new SimpleMotorFeedforward(
          MODULE.ksDriveVoltsRotation,
          MODULE.kvDriveVoltSecondsPerRotation,
          MODULE.kaDriveVoltSecondsSquaredPerRotation);

  private SwerveModuleVisualizer m_moduleVisualizer;

  private TalonFXSimState m_turnMotorSimState;
  private TalonFXSimState m_driveMotorSimState;
  private CANcoderSimState m_angleEncoderSimState;

  private DCMotorSim m_turnMotorSim =
      new DCMotorSim(MODULE.kTurnGearbox, MODULE.kTurnMotorGearRatio, 0.5);
  private DCMotorSim m_driveMotorSim =
      new DCMotorSim(
          //          LinearSystemId.createDCMotorSystem(0.134648227, 0.002802309),
          LinearSystemId.createDCMotorSystem(0.02, 0.001),
          MODULE.kDriveGearbox,
          MODULE.kDriveMotorGearRatio);

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
    var driveConfig = CtreUtils.generateDriveMotorConfig();
    if (invertDirection) {
      turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    configureTalonFx(m_turnMotor, turnConfig);
    configureTalonFx(m_driveMotor, driveConfig);
    setTurnAngle(0);

    m_lastHeadingR2d = getTurnHeadingR2d();
    m_moduleVisualizer = new SwerveModuleVisualizer(this.getName(), DRIVE.kMaxSpeedMetersPerSecond);

    initSmartDashboard();

    // To distinguish modules in CommandScheduler
    setName("SwerveModule_" + m_modulePosition.ordinal());

    if (!RobotBase.isReal()) {
      m_turnMotorSimState = m_turnMotor.getSimState();
      m_driveMotorSimState = m_driveMotor.getSimState();
      m_angleEncoderSimState = m_angleEncoder.getSimState();
    }

    SmartDashboard.putData(
        "SwerveModule2D_" + m_modulePosition.ordinal(), m_moduleVisualizer.getMechanism2d());
  }

  public boolean getInitSuccess() {
    return m_initSuccess;
  }

  public MODULE_POSITION getModulePosition() {
    return m_modulePosition;
  }

  public Rotation2d getTurnEncoderAbsHeading() {
    m_angleEncoder.getAbsolutePosition().refresh();
    return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
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
    return 360.0 * m_turnMotor.getPosition().getValue();
  }

  public Rotation2d getTurnHeadingR2d() {
    return Rotation2d.fromDegrees(getTurnHeadingDeg());
  }

  public double getDriveMps() {
    return m_driveMotor.getVelocity().getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public double getDriveMeters() {
    return m_driveMotor.getPosition().getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    m_desiredState = SwerveModuleState.optimize(desiredState, getTurnHeadingR2d());

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    m_desiredState.speedMetersPerSecond *= m_desiredState.angle.minus(getTurnHeadingR2d()).getCos();

    if (isOpenLoop) {
      double percentOutput = m_desiredState.speedMetersPerSecond / DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.setControl(driveMotorDutyControl.withOutput(percentOutput));
    } else {
      double velocityRPS =
          m_desiredState.speedMetersPerSecond / (MODULE.kWheelDiameterMeters * Math.PI);
      m_driveMotor.setControl(
          driveVelocityControl
              .withVelocity(velocityRPS)
              .withFeedForward(m_driveFF.calculate(m_desiredState.speedMetersPerSecond)));
    }

    var heading =
        (Math.abs(m_desiredState.speedMetersPerSecond) <= (DRIVE.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastHeadingR2d
            : m_desiredState.angle; // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.setControl(
        turnPositionControl
            .withPosition(heading.getRotations())
            .withFeedForward(m_turnFF.calculate(heading.getRotations())));
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

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMps(), getTurnHeadingR2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getTurnHeadingR2d());
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
        String.format("Swerve/Module %d/Turn Motor Desired Position", m_modulePosition.ordinal()),
        m_desiredState.angle.getDegrees());
    Logger.recordOutput(
        String.format("Swerve/Module %d/Turn Motor Position", m_modulePosition.ordinal()),
        getTurnHeadingDeg());
    Logger.recordOutput(
        String.format("Swerve/Module %d/Drive Motor Desired Velocity", m_modulePosition.ordinal()),
        m_desiredState.speedMetersPerSecond);
    Logger.recordOutput(
        String.format("Swerve/Module %d/Drive Motor Velocity", m_modulePosition.ordinal()),
        getDriveMps());

    // Debug
    //    Logger.recordOutput(
    //        String.format("Swerve/Module %d/Drive Motor Setpoint", m_modulePosition.ordinal()),
    //        Double.valueOf(
    //                m_driveMotor.getAppliedControl().getControlInfo().getOrDefault("Velocity",
    // "0"))
    //            * (MODULE.kWheelDiameterMeters * Math.PI));
    //    Logger.recordOutput(
    //        String.format("Swerve/Module %d/Drive Motor Error", m_modulePosition.ordinal()),
    //        m_driveMotor.getClosedLoopError().getValue() * (MODULE.kWheelDiameterMeters *
    // Math.PI));

    //    Logger.recordOutput(
    //        String.format("Swerve/Module %d/Turn Motor Setpoint", m_modulePosition.ordinal()),
    //        Double.valueOf(
    //            m_turnMotor.getAppliedControl().getControlInfo().getOrDefault("Position", "0")) *
    // 360.0);
    //    Logger.recordOutput(
    //        String.format("Swerve/Module %d/Turn Motor Error", m_modulePosition.ordinal()),
    //        m_turnMotor.getClosedLoopError().getValue());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    if (!ROBOT.disableLogging) updateLog();

    if (!ROBOT.disableVisualization) m_moduleVisualizer.update(getState());
  }

  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_turnMotorSim.getCurrentDrawAmps(), m_driveMotorSim.getCurrentDrawAmps()));

    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12, 12));
    m_driveMotorSim.setInputVoltage(
        MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));

    double dt = RobotTime.getTimeDelta();
    m_turnMotorSim.update(dt);
    m_driveMotorSim.update(dt);

    m_turnMotorSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_turnMotorSim.getCurrentDrawAmps() * kMotorResistance);
    m_driveMotorSimState.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - m_driveMotorSim.getCurrentDrawAmps() * kMotorResistance);

    var turnVelocityRps = m_turnMotorSim.getAngularVelocityRPM() / 60.0;
    var driveVelocityRps =
        m_driveMotorSim.getAngularVelocityRPM() * MODULE.kDriveMotorGearRatio / 60.0;

    m_turnMotorSimState.setRawRotorPosition(
        m_turnMotorSim.getAngularPositionRotations() * MODULE.kTurnMotorGearRatio);
    m_turnMotorSimState.setRotorVelocity(turnVelocityRps * MODULE.kTurnMotorGearRatio);
    m_angleEncoderSimState.setRawPosition(m_turnMotorSim.getAngularPositionRotations());
    m_angleEncoderSimState.setVelocity(turnVelocityRps);
    m_driveMotorSimState.setRawRotorPosition(
        m_driveMotorSim.getAngularPositionRotations() * MODULE.kDriveMotorGearRatio);
    m_driveMotorSimState.setRotorVelocity(driveVelocityRps);
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_turnMotor.close();
    m_angleEncoder.close();
  }
}
