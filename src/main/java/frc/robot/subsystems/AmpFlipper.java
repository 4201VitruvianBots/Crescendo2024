// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AMP;
import frc.robot.constants.CAN;
import frc.robot.utils.CtreUtils;

public class AmpFlipper extends SubsystemBase {
  /** Creates a new AmpFlipper. */
  private final TalonFX flipperMotor = new TalonFX(CAN.ampFlipper);

  private final PositionVoltage m_position = new PositionVoltage(0);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(AMP.kMaxFlipperVelocity, AMP.kMaxFlipperAcceleration);
  private final TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private double m_desiredAngleRadians = 0;
  
  private Timer m_simTimer = new Timer();
  private double lastSimTime;

  // Simulation setup
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          AMP.gearBox,
          AMP.gearRatio,
          SingleJointedArmSim.estimateMOI(AMP.length, AMP.mass),
          AMP.length,
          AMP.minAngle,
          AMP.maxAngle,
          false,
          AMP.fourbarAngleDegrees
          //VecBuilder.fill(2.0 * Math.PI / 2048.0) // Add noise with a std-dev of 1 tick
          );
  
  private final Mechanism2d m_amp2d = new Mechanism2d(AMP.length*2, AMP.length*2);

  private final MechanismRoot2d m_ampRoot2d = m_amp2d.getRoot("Amp Flipper", AMP.length/2, AMP.length/2);
  
  private final MechanismLigament2d m_ampLigament2d =
      new MechanismLigament2d("Amp Flipper", AMP.length, AMP.fourbarAngleDegrees);
  
  public AmpFlipper() {
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kS = AMP.kS;
    config.Slot0.kV = AMP.kV;
    config.Slot0.kP = AMP.kP;
    config.Slot0.kI = AMP.kI;
    config.Slot0.kD = AMP.kD;
    CtreUtils.configureTalonFx(flipperMotor, config);
    
    // Simulation setup
    lastSimTime = m_simTimer.get();
    m_ampRoot2d.append(m_ampLigament2d);
  }

  public void setPercentOutput(double speed) {
    flipperMotor.set(speed);
  }

  public void setDesiredSetpointRadians(double radians) {
    m_desiredAngleRadians = radians;
    m_goal = new TrapezoidProfile.State(radians * AMP.kRadiansToRotations, 0);
  }

  public double getDesiredSetpointRadians() {
    return m_desiredAngleRadians;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // periodic, update the profile setpoint for 20 ms loop time
    m_setpoint = m_profile.calculate(RobotTime.getTimeDelta(), m_setpoint, m_goal);
    // apply the setpoint to the control request
    m_position.Position = m_setpoint.position;
    m_position.Velocity = m_setpoint.velocity;
    flipperMotor.setControl(m_position);
  }
  
  @Override
  public void simulationPeriodic() {
    m_armSim.setInputVoltage(MathUtil.clamp(flipperMotor.getMotorVoltage().getValueAsDouble(), -12, 12));

    double dt = m_simTimer.get() - lastSimTime;
    m_armSim.update(dt);
    lastSimTime = m_simTimer.get();

    Unmanaged.feedEnable(20);

    // Using negative sensor units to match physical behavior
    flipperMotor
        .setPosition(
            (int)
                (Units.radiansToDegrees(m_armSim.getAngleRads())
                    / AMP.rotationsToDegrees));

    flipperMotor
        .setVoltage(
            (int)
                (Units.radiansToDegrees(m_armSim.getVelocityRadPerSec())
                    / AMP.rotationsToDegrees
                    * 10.0));
    
    // Set supply voltage of flipper motor
    //flipperMotor.setBusVoltage(RobotController.getBatteryVoltage());
    m_ampLigament2d.setAngle(m_armSim.getAngleRads());
    // Ligma2d?
  }
  
}
