// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;

public class Uptake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX uptakeMotor = new TalonFX(CAN.uptakeMotor);
  
  NetworkTable uptakeNtTab =
    NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Uptake");

  DoublePublisher uptakeSpeed;
  
  public Uptake() {
    uptakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    
    uptakeSpeed = uptakeNtTab.getDoubleTopic("Uptake Speed").publish();
  }

  public void setSpeed(double speed) {
    uptakeMotor.set(speed);
  }
  
  public void updateSmartDashboard() {
    uptakeSpeed.set(uptakeMotor.get());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}