// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ARM;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final TalonFX[] elevatorClimbMotors = {
    new TalonFX(CLIMBER.climbMotor1), new TalonFX(CLIMBER.climbMotor2)
  };
  private final StaticBrake brake = new StaticBrake();
  private final Follower follower = new Follower(0, false);

  // Trapezoid profile setup
  public final PositionVoltage m_position = new PositionVoltage(0);

  public TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(CLIMBER.kMaxVel, CLIMBER.kMaxAccel);
  private final TrapezoidProfile m_currentProfile = new TrapezoidProfile(m_constraints);

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(CLIMBER.kG, CLIMBER.kV, CLIMBER.kA);

  private double m_desiredPositionMeters; // The height in meters our robot is trying to reach
  private double m_upperLimitMeters = CLIMBER.upperLimitMeters;
  private double m_lowerLimitMeters = CLIMBER.lowerLimitMeters;
  private CLIMBER_SETPOINT m_desiredSetpoint = CLIMBER_SETPOINT.FULL_RETRACT;

  private CONTROL_MODE m_controlMode = CONTROL_MODE.CLOSED_LOOP;
  // Controlled by open loop
  private double m_joystickInput;
  private boolean m_limitJoystickInput;
  private boolean m_userSetpoint;

  private boolean elevatorClimbSate;
  private double holdPosition;

  private TalonFXSimState m_simState = elevatorClimbMotors[0].getSimState();
  // Testing value for mech2d
  public double m_mechHeight = 0.5;

  public DoubleSubscriber m_mechHeightSub;
  public double m_simEncoderSign = 1;
  public final ElevatorSim elevatorSim =
      new ElevatorSim(
          CLIMBER.gearbox,
          CLIMBER.gearRatio,
          3.0,
          CLIMBER.drumRadiusMeters,
          CLIMBER.lowerLimitMeters,
          CLIMBER.upperLimitMeters,
          true,
          CLIMBER.lowerLimitMeters);

  NetworkTable climberNtTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Climber");

  /** Creates a new climberMechanism. */
  public Climber() {
    // Initialize Test Values
    climberNtTab.getDoubleTopic("Climber Sim Test Height").publish().set(m_mechHeight);
    m_mechHeightSub =
        climberNtTab.getDoubleTopic("Climber Sim Test Height").subscribe(m_mechHeight);

    for (TalonFX motor : elevatorClimbMotors) motor.setControl(brake);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = ARM.kP;
    config.Slot0.kI = ARM.kI;
    config.Slot0.kD = ARM.kD;

    elevatorClimbMotors[0].getConfigurator().apply(new TalonFXConfiguration());
    elevatorClimbMotors[0].setInverted(true);
    elevatorClimbMotors[1].setInverted(true);
    elevatorClimbMotors[1].setControl(follower);

    m_simEncoderSign = elevatorClimbMotors[0].getInverted() ? -1 : 1;
  }

  public void setClimbState(boolean state) {
    elevatorClimbSate = state;
  }

  public boolean getClimbState() {
    return elevatorClimbSate;
  }

  public double getPercentOutput() {
    return elevatorClimbMotors[0].getDutyCycle().getValueAsDouble();
  }

  // sets the percent ourput of the elevator based on its position
  public void setPercentOutput(double output, boolean enforceLimits) {
    if (enforceLimits) {
      if (getHeightMeters() > getUpperLimitMeters() - Units.inchesToMeters(1.2))
        output = Math.min(output, 0);

      if (getHeightMeters() < getLowerLimitMeters() + Units.inchesToMeters(0.05))
        output = Math.max(output, 0);
    }

    elevatorClimbMotors[0].set(output);
  }

  // Sets the calculated trapezoid state of the motors
  public void setSetpointTrapezoidState(double rotations) {
    m_desiredPositionMeters = rotations;
    m_goal = new TrapezoidProfile.State(m_desiredPositionMeters, 0);
  }

  private double calculateFeedforward(TrapezoidProfile.State state) {
    return (m_feedForward.calculate(state.position, state.velocity) / 12.0);
  }

  // Sets the setpoint to our current height, effectively keeping the elevator in place.
  public void resetTrapezoidState() {
    m_setpoint = new TrapezoidProfile.State(getHeightMeters(), getVelocityMetersPerSecond());
  }

  public double getVelocityMetersPerSecond() {
    return elevatorClimbMotors[0].getRotorVelocity().getValueAsDouble()
        * CLIMBER.encoderCountsToMeters
        * 10;
  }

  // gets the position of the climber in meters
  public double getHeightMeters() {
    return getHeightEncoderCounts() * CLIMBER.encoderCountsToMeters;
  }

  // gets the position of the climber in encoder counts
  public double getHeightEncoderCounts() {
    return elevatorClimbMotors[0].getRotorPosition().getValueAsDouble();
  }

  // sets position in meters
  public void setSensorPosition(double meters) {
    elevatorClimbMotors[0].setPosition(meters / CLIMBER.encoderCountsToMeters);
  }

  public void holdClimber() {
    elevatorClimbMotors[0].set(holdPosition);
  }

  public void setDesiredSetpoint(CLIMBER_SETPOINT desiredSetpoint) {
    m_desiredSetpoint = desiredSetpoint;
    setCLimberDesiredSetpoint(desiredSetpoint);
  }

  public void setCLimberDesiredSetpoint(CLIMBER_SETPOINT desiredSetpoint) {
    desiredSetpoint.getClimberSetpointMeters();
  }

  public void setDesiredPositionMeters(double meters) {
    m_desiredPositionMeters = meters;
  }

  public double getDesiredPostionMeters(double meters) {
    return m_desiredPositionMeters;
  }

  public void setHoldPosition(double position) {
    holdPosition = position;
  }

  public void setLowerLimitMeters(double meters) {
    m_lowerLimitMeters = meters;
  }

  public double getLowerLimitMeters() {
    return m_lowerLimitMeters;
  }

  public void setUpperLimitMeters(double meters) {
    m_upperLimitMeters = meters;
  }

  public double getUpperLimitMeters() {
    return m_upperLimitMeters;
  }

  public void setJoystickLimit(boolean limit) {
    m_limitJoystickInput = limit;
  }

  public void setJoystickY(double m_joystickY) {
    m_joystickInput = m_joystickY;
  }

  public boolean isUserControlled() {
    return m_joystickInput != 0 && !m_userSetpoint;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }

  // Sets the control state of the elevator
  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  // Returns the current control state enum
  public CONTROL_MODE getClosedLoopControlMode() {
    return m_controlMode;
  }

  public boolean isClosedLoopControl() {
    return getClosedLoopControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setClimberNeutralMode(NeutralModeValue mode) {
    elevatorClimbMotors[0].setNeutralMode(mode);
    elevatorClimbMotors[1].setNeutralMode(mode);
  }

  private void updateLogger() {
    Logger.recordOutput("Climber/CONTROL_MODE", getClosedLoopControlMode());
    Logger.recordOutput("Climber/Height Meters", getHeightMeters());
    Logger.recordOutput("Climber/Height Encoder Counts", getHeightEncoderCounts());
    Logger.recordOutput("Climber/Climb State", getClimbState());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (m_controlMode) {
      case OPEN_LOOP:
        double percentOutput = m_joystickInput * CLIMBER.kPercentOutputMultiplier;

        if (m_limitJoystickInput)
          percentOutput = m_joystickInput * CLIMBER.kLimitedPercentOutputMultiplier;

        setPercentOutput(percentOutput, true);
        break;
      default:
      case CLOSED_LOOP:
        // Updates our trapezoid profile state based on the time since our last periodic and our
        // recorded change in height
        m_setpoint = m_currentProfile.calculate(RobotTime.getTimeDelta(), m_setpoint, m_goal);
        m_position.Position = m_setpoint.position;
        m_position.Velocity = m_setpoint.velocity;

        elevatorClimbMotors[0].setControl(m_position);
        break;
    }
    updateLogger();
  }

  @Override
  public void simulationPeriodic() {

    m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    elevatorSim.setInputVoltage(
        MathUtil.clamp(elevatorClimbMotors[0].getMotorVoltage().getValueAsDouble(), -12, 12));

    elevatorSim.update(RobotTime.getTimeDelta());

    elevatorSim.update(RobotTime.getTimeDelta());

    m_simState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * CLIMBER.gearRatio);

    m_simState.setRawRotorPosition(elevatorSim.getPositionMeters() * CLIMBER.gearRatio);
  }
}
