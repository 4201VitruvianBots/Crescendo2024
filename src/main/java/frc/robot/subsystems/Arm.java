// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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

  private final StatusSignal<Double> m_positionSignal = m_armMotor.getPosition().clone();

  private double m_desiredRotations = ARM.ARM_SETPOINT.STOWED.get();

  private final PositionVoltage m_position = new PositionVoltage(m_desiredRotations);

  private TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(ARM.kMaxArmVelocity, ARM.kMaxArmAcceleration);
  private final TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

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
      m_kP_subscriber,
      m_kI_subscriber,
      m_kD_subscriber,
      m_kMaxArmVelocity_subscriber,
      m_kMaxArmAcceleration_subscriber,
      m_kSetpoint_subscriber;
  private final NetworkTable armTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Arm");

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ARM.gearRatio;
    config.Slot0.kS = ARM.kS;
    config.Slot0.kV = ARM.kV;
    config.Slot0.kP = ARM.kP;
    config.Slot0.kI = ARM.kI;
    config.Slot0.kD = ARM.kD;
    config.MotorOutput.PeakForwardDutyCycle = 0.2;
    config.MotorOutput.PeakReverseDutyCycle = -0.2;
    CtreUtils.configureTalonFx(m_armMotor, config);

    // Simulation setup
    SmartDashboard.putData(this);

    m_armMotor.setPosition(Units.degreesToRotations(ARM.startingAngleDegrees));
  }

  // Get the percent output of the arm motor.
  public void setPercentOutput(double speed) {
    m_armMotor.set(speed);
  }

  public double getPercentOutput() {
    return m_armMotor.get();
  }

  public void setDesiredSetpointRotations(double rotations) {
    m_desiredRotations =
        MathUtil.clamp(
            rotations,
            Units.degreesToRotations(ARM.minAngleDegrees),
            Units.degreesToRotations(ARM.maxAngleDegrees));
    m_goal = new TrapezoidProfile.State(m_desiredRotations, 0);
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

  public void setControlMode(ROBOT.CONTROL_MODE mode) {
    if (mode == ROBOT.CONTROL_MODE.CLOSED_LOOP && m_controlMode == ROBOT.CONTROL_MODE.OPEN_LOOP)
      resetTrapezoidState();
    m_controlMode = mode;
  }

  public ROBOT.CONTROL_MODE getControlMode() {
    return m_controlMode;
  }

  public void resetSensorPosition() {
    if (RobotBase.isReal()) {
      m_armMotor.setPosition(Units.degreesToRotations(ARM.startingAngleDegrees));
      resetTrapezoidState();
    } else {
      m_simState.setRawRotorPosition(Units.degreesToRotations(ARM.startingAngleDegrees));
      resetTrapezoidState();
    }
  }

  public void resetTrapezoidState() {
    m_setpoint = new TrapezoidProfile.State(getCurrentRotation(), 0);
  }

  public TalonFX getMotor() {
    return m_armMotor;
  }

  private void updateLogger() {
    Logger.recordOutput("Arm/ControlMode", m_controlMode.toString());
    Logger.recordOutput("Arm/CurrentAngle", getCurrentAngle());
    Logger.recordOutput("Arm/DesiredAngle", Units.rotationsToDegrees(m_desiredRotations));
    Logger.recordOutput("Arm/DesiredSetpoint", Units.rotationsToDegrees(m_goal.position));
    Logger.recordOutput("Arm/PercentOutput", m_armMotor.get());
    Logger.recordOutput("Arm/CurrentOutput", m_armMotor.getTorqueCurrent().getValue());
  }

  public void testInit() {
    armTab.getDoubleTopic("kS").publish().set(ARM.kS);
    armTab.getDoubleTopic("kV").publish().set(ARM.kV);
    armTab.getDoubleTopic("kP").publish().set(ARM.kP);
    armTab.getDoubleTopic("kI").publish().set(ARM.kI);
    armTab.getDoubleTopic("kD").publish().set(ARM.kD);

    armTab.getDoubleTopic("kMaxVel").publish().set(ARM.kMaxArmVelocity);
    armTab.getDoubleTopic("kMaxAccel").publish().set(ARM.kMaxArmAcceleration);

    armTab.getDoubleTopic("kSetpoint").publish().set(getCurrentAngle());

    m_kS_subscriber = armTab.getDoubleTopic("kS").subscribe(ARM.kS);
    m_kV_subscriber = armTab.getDoubleTopic("kV").subscribe(ARM.kV);
    m_kP_subscriber = armTab.getDoubleTopic("kP").subscribe(ARM.kP);
    m_kI_subscriber = armTab.getDoubleTopic("kI").subscribe(ARM.kI);
    m_kD_subscriber = armTab.getDoubleTopic("kD").subscribe(ARM.kD);

    m_kMaxArmVelocity_subscriber = armTab.getDoubleTopic("kMaxVel").subscribe(ARM.kMaxArmVelocity);
    m_kMaxArmAcceleration_subscriber =
        armTab.getDoubleTopic("kMaxAccel").subscribe(ARM.kMaxArmAcceleration);

    m_kSetpoint_subscriber =
        armTab.getDoubleTopic("kSetpoint").subscribe(Units.rotationsToDegrees(m_desiredRotations));
  }

  public void testPeriodic() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = m_kS_subscriber.get(ARM.kS);
    slot0Configs.kV = m_kV_subscriber.get(ARM.kV);
    slot0Configs.kP = m_kP_subscriber.get(ARM.kP);
    slot0Configs.kI = m_kI_subscriber.get(ARM.kI);
    slot0Configs.kD = m_kD_subscriber.get(ARM.kD);

    m_armMotor.getConfigurator().apply(slot0Configs);

    m_constraints =
        new TrapezoidProfile.Constraints(
            m_kMaxArmVelocity_subscriber.get(), m_kMaxArmAcceleration_subscriber.get());

    double m_oldSetpoint = Units.rotationsToDegrees(m_desiredRotations);
    m_desiredRotations =
        Units.degreesToRotations(
            m_kSetpoint_subscriber.get(Units.rotationsToDegrees(m_desiredRotations)));
    if (m_desiredRotations != m_oldSetpoint) setDesiredSetpointRotations(m_desiredRotations);
  }

  public void teleopInit() {
    resetTrapezoidState();
    setDesiredSetpointRotations(getCurrentRotation());
  }

  @Override
  public void periodic() {
    switch (m_controlMode) {
      case CLOSED_LOOP:
        // This method will be called once per scheduler run
        // periodic, update the profile setpoint for 20 ms loop time
        m_setpoint = m_profile.calculate(RobotTime.getTimeDelta(), m_setpoint, m_goal);
        // apply the setpoint to the control request
        m_position.Position = m_setpoint.position;
        m_position.Velocity = m_setpoint.velocity;
        if (DriverStation.isEnabled()) m_armMotor.setControl(m_position);
        break;
      default:
      case OPEN_LOOP:
        if (DriverStation.isDisabled()) {
          setPercentOutput(0.0);
        }
        break;
    }

    if (!ROBOT.disableLogging) updateLogger();
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
  }
}
