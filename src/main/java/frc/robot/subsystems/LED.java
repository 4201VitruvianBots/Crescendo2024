/* Copyright (c) FIRST and other WPILib contributors.
   Open Source Software; you can modify and/or share it under the terms of
   the WPILib BSD license file in the root directory of this project. */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import frc.robot.constants.CAN;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  
  private final CANdle m_candle = new CANdle(CAN.CANdle);
  private int red = 0;
  private int green = 0; // setting all LED colors to none: there is no color when robot activates
  private int blue = 0;
  // TODO: Use enum for robot state
  private boolean setSolid;
  private Animation m_toAnimate = null;
  
  public LED(Controls controls) {
    m_candle.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
