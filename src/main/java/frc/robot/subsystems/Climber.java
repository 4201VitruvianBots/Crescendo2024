// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private boolean isClimbing = false;

  // Testing value for mech2d
  public double m_mechHeight = 0.1;

  public DoubleSubscriber m_mechHeightSub;

  NetworkTable climberNtTab =
      NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Climber");

  public Climber() {
    // Initialize Test Values
    climberNtTab.getDoubleTopic("Climber Sim Test Height").publish().set(m_mechHeight);
    m_mechHeightSub =
        climberNtTab.getDoubleTopic("Climber Sim Test Height").subscribe(m_mechHeight);
  }

  public boolean getClimberState() {
    return isClimbing;
  }

  public void setClimberState(boolean state) {
    isClimbing = state;
  }

  public double getHeightMeters() {
    return m_mechHeight;
  }

  public double getPercentOutput() {
    // TODO: Implement
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_mechHeight = m_mechHeightSub.get();
  }
}
