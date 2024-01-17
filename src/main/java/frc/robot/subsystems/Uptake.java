// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.CAN.UPTAKE_STATE;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Uptake extends SubsystemBase {
  /** Creates a new Uptake. */
  private boolean m_isUptaking = false;

 
  private UPTAKE_STATE m_state = UPTAKE_STATE.NONE;

  private final TalonFX uptakeMotor = new TalonFX(CAN.uptakeMotor);

  public Uptake() {
    CtreUtils.configureTalonFx(uptakeMotor, new TalonFXConfiguration());
  }

  public void setSpeed(double speed) {
    uptakeMotor.set(speed);
  }
   // control mode function
   public void setUptaking(boolean isUptaking) {
    m_isUptaking = isUptaking;
  }

  public boolean isUptaking() {
    return m_isUptaking;
  }

  public void setUptakingState(UPTAKE_STATE speed) {
    m_state = speed;
  }

  public UPTAKE_STATE getUptakeState() {
    return m_state;
  }
  public void updateSmartDashboard() {}

  public void updateLog() {
    Logger.recordOutput("Uptake/Motor Speed", uptakeMotor.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    updateLog();
  }
}
