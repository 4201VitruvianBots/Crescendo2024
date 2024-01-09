// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.CtreUtils.configureCANCoder;
import static frc.robot.utils.CtreUtils.configureTalonFx;
import static frc.robot.utils.ModuleMap.MODULE_POSITION;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.SWERVE.MODULE;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModuleMap;
import frc.robot.visualizers.SwerveModuleVisualizer;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase implements AutoCloseable {
  private final ModuleMap.MODULE_POSITION m_modulePosition;
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_angleEncoder;

  private final double m_angleOffset;
  private Rotation2d m_lastHeadingR2d;
  private Pose2d m_pose;
  private boolean m_initSuccess = false;
  private SwerveModuleState m_desiredState;

  private final DutyCycleOut driveMotorDutyControl = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0);
  private final PositionVoltage turnPositionControl = new PositionVoltage(0);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          MODULE.ksDriveVoltSecondsPerMeter,
          MODULE.kvDriveVoltSecondsSquaredPerMeter,
          MODULE.kaDriveVoltSecondsSquaredPerMeter);

  private SwerveModuleVisualizer m_moduleVisualizer;

  private TalonFXSimState m_turnMotorSimState;
  private TalonFXSimState m_driveMotorSimState;
  private CANcoderSimState m_angleEncoderSimState;

  private DCMotorSim m_turnMotorSim =
      new DCMotorSim(MODULE.kTurnGearbox, MODULE.kTurnMotorGearRatio, 0.5);
  private DCMotorSim m_driveMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(0.134648227, 0.002802309),
          MODULE.kDriveGearbox,
          MODULE.kDriveMotorGearRatio);

  public SwerveModule(
      MODULE_POSITION modulePosition,
      TalonFX turnMotor,
      TalonFX driveMotor,
      CANcoder angleEncoder,
      double angleOffset) {
    m_modulePosition = modulePosition;
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = RobotBase.isReal() ? angleOffset : 0;

    if (RobotBase.isSimulation()) {
      m_angleEncoder.setPosition(0);
    }
    configureCANCoder(m_angleEncoder, CtreUtils.generateCanCoderConfig());
    m_angleEncoder.optimizeBusUtilization(255);
    configureTalonFx(m_turnMotor, CtreUtils.generateTurnMotorConfig());
    configureTalonFx(m_driveMotor, CtreUtils.generateDriveMotorConfig());
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
    m_driveMotor.setControl(new StaticBrake());
  }

  public void setDriveNeutral() {
    m_driveMotor.setControl(new NeutralOut());
  }

  public void setTurnBrake() {
    m_turnMotor.setControl(new StaticBrake());
  }

  public void setTurnCoast() {
    m_turnMotor.setControl(new NeutralOut());
  }

  private void initSmartDashboard() {}

  private void updateSmartDashboard() {}

  public void updateLog() {
    Logger.recordOutput(String.format("Swerve/Module %d/Encoder Absolute Position", m_modulePosition.ordinal()), getTurnEncoderAbsHeading());
    Logger.recordOutput(String.format("Swerve/Module %d/Turn Motor Position", m_modulePosition.ordinal()), getTurnHeadingDeg());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();

    m_moduleVisualizer.update(getState());
  }

  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_turnMotorSim.getCurrentDrawAmps(), m_driveMotorSim.getCurrentDrawAmps()));

    m_turnMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12, 12));
    m_driveMotorSim.setInputVoltage(
        MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));

    double dt = RobotTime.getTimeDelta();
    m_turnMotorSim.update(dt);
    m_driveMotorSim.update(dt);

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
