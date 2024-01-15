// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.UPTAKE.UPTAKE_STATE;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class Uptake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX uptakeMotor = new TalonFX(CAN.uptakeMotor);

  public Uptake() {
    CtreUtils.configureTalonFx(uptakeMotor, new TalonFXConfiguration());
  }

  public void setSpeed(double d) {
    uptakeMotor.set(d);
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
