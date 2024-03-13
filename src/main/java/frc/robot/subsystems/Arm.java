// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ARM;
import frc.robot.constants.CAN;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX m_armMotor = new TalonFX(CAN.armMotor);

  private final TalonFXSimState m_simState = m_armMotor.getSimState();

  private final CANcoder m_armEncoder = new CANcoder(CAN.armCanCoder);
  private final CANcoderSimState m_armEncoderSimState = m_armEncoder.getSimState();

  private final StatusSignal<Double> m_positionSignal = m_armMotor.getPosition().clone();
  private final StatusSignal<Double> m_currentSignal = m_armMotor.getTorqueCurrent().clone();

  private TorqueCurrentFOC m_torqueCurrentFOC = new TorqueCurrentFOC(0);
  private NeutralModeValue m_neutralMode = NeutralModeValue.Brake;

  private double m_desiredRotations = ARM.ARM_SETPOINT.STOWED.get();

  private final MotionMagicTorqueCurrentFOC m_request =
      new MotionMagicTorqueCurrentFOC(getCurrentRotation());

  // Simulation setup
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          ARM.gearBox,
          ARM.gearRatio,
          SingleJointedArmSim.estimateMOI(ARM.armLength, ARM.mass),
          ARM.armLength,
          Units.degreesToRadians(ARM.minAngleDegrees),
          Units.degreesToRadians(ARM.maxAngleDegrees - ARM.minAngleDegrees),
          false,
          Units.degreesToRadians(ARM.startingAngleDegrees));

  private ROBOT.CONTROL_MODE m_controlMode = ROBOT.CONTROL_MODE.CLOSED_LOOP;

  // Test mode setup
  private DoubleSubscriber m_kS_subscriber,
      m_kV_subscriber,
      m_kA_subscriber,
      m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_kAccel_subscriber,
      m_kCruiseVel_subscriber,
      m_kJerk_subscriber,
      m_kSetpoint_subscriber;
  private final NetworkTable armTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm");

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = m_neutralMode;
    config.Feedback.RotorToSensorRatio = ARM.gearRatio;
    config.Feedback.FeedbackRemoteSensorID = CAN.armCanCoder;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Slot0.kS = ARM.kS;
    config.Slot0.kV = ARM.kV;
    config.Slot0.kP = ARM.kP;
    config.Slot0.kI = ARM.kI;
    config.Slot0.kD = ARM.kD;
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.MotorOutput.PeakForwardDutyCycle = ARM.maxOutput;
    config.MotorOutput.PeakReverseDutyCycle = -ARM.maxOutput;

    config.MotionMagic.MotionMagicAcceleration = ARM.kAccel;
    config.MotionMagic.MotionMagicCruiseVelocity = ARM.kCruiseVel;
    config.MotionMagic.MotionMagicJerk = ARM.kJerk;
    CtreUtils.configureTalonFx(m_armMotor, config);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = ARM.canCoderOffset;
    CtreUtils.configureCANCoder(m_armEncoder, canCoderConfig);

    m_armEncoder.setPosition(m_armEncoder.getPosition().getValue());

    SmartDashboard.putData(this);
  }

  // Get the percent output of the arm motor.
  public void setPercentOutput(double speed) {
    m_armMotor.set(speed);
  }

  public void setFocOutput(double output) {
    m_armMotor.setControl(m_torqueCurrentFOC.withOutput(output * 327.0));
  }

  public double getPercentOutput() {
    return m_armMotor.getMotorVoltage().getValue() / 12.0;
  }

  public void setDesiredSetpointRotations(double rotations) {
    m_desiredRotations =
        MathUtil.clamp(
            rotations,
            Units.degreesToRotations(ARM.minAngleDegrees),
            Units.degreesToRotations(ARM.maxAngleDegrees));
  }

  public double getDesiredSetpointRotations() {
    return m_desiredRotations;
  }

  public double getCurrentRotation() {
    m_positionSignal.refresh();
    return m_positionSignal.getValue();
  }

  public double getCurrentAngle() {
    return Units.rotationsToDegrees(getCurrentRotation());
  }

  public double getCANcoderAngle() {
    return m_armEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public void setNeutralMode(NeutralModeValue mode) {
    if (mode == m_neutralMode) return;
    m_neutralMode = mode;
    m_armMotor.setNeutralMode(mode);
  }

  public void setControlMode(ROBOT.CONTROL_MODE mode) {
    if (mode == ROBOT.CONTROL_MODE.CLOSED_LOOP && m_controlMode == ROBOT.CONTROL_MODE.OPEN_LOOP)
      resetMotionMagicState();
    m_controlMode = mode;
  }

  public ROBOT.CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  public void resetSensorPositionHome() {
    resetSensorPosition(ARM.startingAngleDegrees);
  }

  public void resetSensorPosition(double m_angle) {
    m_armMotor.setPosition(Units.degreesToRotations(m_angle));
    resetMotionMagicState();
  }

  public void resetMotionMagicState() {
    m_desiredRotations = getCurrentRotation();
    m_armMotor.setControl(m_request.withPosition(m_desiredRotations));
  }

  public TalonFX getMotor() {
    return m_armMotor;
  }

  public SingleJointedArmSim getSim() {
    return m_armSim;
  }

  public double getInputVoltage() {
    return m_armMotor.getMotorVoltage().getValue();
  }

  public double getRotationalVelocity() {
    return m_armMotor.getVelocity().getValue();
  }

  private void updateLogger() {
    Logger.recordOutput("Arm/ControlMode", m_controlMode.toString());
    Logger.recordOutput("Arm/CurrentAngle", getCurrentAngle());
    Logger.recordOutput("Arm/CurrentOutput", m_currentSignal.getValue());
    Logger.recordOutput("Arm/DesiredAngle", Units.rotationsToDegrees(m_desiredRotations));
    Logger.recordOutput("Arm/PercentOutput", getPercentOutput());
    Logger.recordOutput("Arm/CanCoderAbsolutePos360", getCANcoderAngle());
  }

  public void testInit() {
    armTab.getDoubleTopic("kS").publish().set(ARM.kS);
    armTab.getDoubleTopic("kV").publish().set(ARM.kV);
    armTab.getDoubleTopic("kA").publish().set(ARM.kA);
    armTab.getDoubleTopic("kP").publish().set(ARM.kP);
    armTab.getDoubleTopic("kI").publish().set(ARM.kI);
    armTab.getDoubleTopic("kD").publish().set(ARM.kD);

    armTab.getDoubleTopic("kAccel").publish().set(ARM.kAccel);
    armTab.getDoubleTopic("kCruiseVel").publish().set(ARM.kCruiseVel);
    armTab.getDoubleTopic("kJerk").publish().set(ARM.kJerk);

    armTab.getDoubleTopic("kSetpoint").publish().set(getCurrentAngle());

    m_kS_subscriber = armTab.getDoubleTopic("kS").subscribe(ARM.kS);
    m_kV_subscriber = armTab.getDoubleTopic("kV").subscribe(ARM.kV);
    m_kA_subscriber = armTab.getDoubleTopic("kA").subscribe(ARM.kA);
    m_kP_subscriber = armTab.getDoubleTopic("kP").subscribe(ARM.kP);
    m_kI_subscriber = armTab.getDoubleTopic("kI").subscribe(ARM.kI);
    m_kD_subscriber = armTab.getDoubleTopic("kD").subscribe(ARM.kD);

    m_kAccel_subscriber = armTab.getDoubleTopic("kAccel").subscribe(ARM.kAccel);
    m_kCruiseVel_subscriber = armTab.getDoubleTopic("kCruiseVel").subscribe(ARM.kCruiseVel);
    m_kJerk_subscriber = armTab.getDoubleTopic("kJerk").subscribe(ARM.kJerk);

    m_kSetpoint_subscriber =
        armTab.getDoubleTopic("kSetpoint").subscribe(Units.rotationsToDegrees(m_desiredRotations));
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = m_kS_subscriber.get(ARM.kS);
    slot0Configs.kV = m_kV_subscriber.get(ARM.kV);
    slot0Configs.kA = m_kA_subscriber.get(ARM.kA);
    slot0Configs.kP = m_kP_subscriber.get(ARM.kP);
    slot0Configs.kI = m_kI_subscriber.get(ARM.kI);
    slot0Configs.kD = m_kD_subscriber.get(ARM.kD);

    m_armMotor.getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    motionMagicConfigs.MotionMagicAcceleration = m_kAccel_subscriber.get(ARM.kAccel);
    motionMagicConfigs.MotionMagicCruiseVelocity = m_kCruiseVel_subscriber.get(ARM.kCruiseVel);
    motionMagicConfigs.MotionMagicJerk = m_kJerk_subscriber.get(ARM.kJerk);

    m_armMotor.getConfigurator().apply(motionMagicConfigs);

    double m_oldSetpoint = Units.rotationsToDegrees(m_desiredRotations);
    m_desiredRotations =
        Units.degreesToRotations(
            m_kSetpoint_subscriber.get(Units.rotationsToDegrees(m_desiredRotations)));
    if (m_desiredRotations != m_oldSetpoint) setDesiredSetpointRotations(m_desiredRotations);
  }

  public void autonomousInit() {
    resetMotionMagicState();
    setDesiredSetpointRotations(getCurrentRotation());
  }

  public void teleopInit() {
    resetMotionMagicState();
    setDesiredSetpointRotations(getCurrentRotation());
  }

  @Override
  public void periodic() {
    switch (m_controlMode) {
      case CLOSED_LOOP:
        // This method will be called once per scheduler run
        // periodic, update the profile setpoint for 20 ms loop time
        m_armMotor.setControl(m_request.withPosition(m_desiredRotations));
        break;
      default:
      case OPEN_LOOP:
        if (DriverStation.isDisabled()) {
          setPercentOutput(0.0);
        }
        break;
    }

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
  }

  @Override
  public void simulationPeriodic() {
    //    // Set supply voltage of flipper motor
    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_armSim.setInputVoltage(MathUtil.clamp(m_simState.getMotorVoltage(), -12, 12));

    m_armSim.update(RobotTime.getTimeDelta());

    m_simState.setRawRotorPosition(
        Units.radiansToRotations(m_armSim.getAngleRads()) * ARM.gearRatio);

    m_simState.setRotorVelocity(
        Units.radiansToRotations(m_armSim.getVelocityRadPerSec()) * ARM.gearRatio);

    m_armEncoderSimState.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads()));
    m_armEncoderSimState.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec()));
  }
}
