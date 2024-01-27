// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class SuperStructureVisualizer {
  Intake m_intake;
  Uptake m_uptake;
  Shooter m_shooter;
  Climber m_climber;
  Vision m_vision;
  

  /* Function to visualize the speed of a particular motor. */
  public void updateMotorColor(MechanismLigament2d ligament, double motorSpeed) {
    double deltaBrightness = Math.abs(motorSpeed) * 75;
    
    Color8Bit m_originalColor = ligament.getColor();
    
    Color8Bit newColor = new Color8Bit(
        m_originalColor.red + (int) deltaBrightness,
        m_originalColor.green + (int) deltaBrightness,
        m_originalColor.blue + (int) deltaBrightness
    );
    
    ligament.setColor(newColor);
  }
  
  /* Function to visualize the state of a limelight. */
  public void updateLimelightColor(MechanismLigament2d ligament, boolean isActive) {
    Color8Bit m_originalColor = ligament.getColor();
    
    Color8Bit newColor = new Color8Bit(
        m_originalColor.red + (isActive ? 75 : 0),
        m_originalColor.green + (isActive ? 75 : 0),
        m_originalColor.blue + (isActive ? 75 : 0)
    );
    
    ligament.setColor(newColor);
  }
  
}
