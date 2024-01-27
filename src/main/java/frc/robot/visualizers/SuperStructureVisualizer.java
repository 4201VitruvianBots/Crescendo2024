// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualizers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.AMP;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.FLYWHEEL;
import frc.robot.constants.INTAKE;
import frc.robot.constants.ROBOT;
import frc.robot.constants.UPTAKE;
import frc.robot.constants.VISION;
import frc.robot.subsystems.*;

/** A class to visualize the state of all mechanisms on the robot. */
public class SuperStructureVisualizer {
  Intake m_intake;
  Uptake m_uptake;
  Shooter m_shooter;
  AmpShooter m_ampShooter;
  Arm m_arm;
  Climber m_climber;
  Vision m_vision;
  
  Mechanism2d m_mech2d = new Mechanism2d(ROBOT.drivebaseLength * 2, ROBOT.drivebaseLength * 2);
  
  MechanismRoot2d m_drivebaseRoot2d = m_mech2d.getRoot("Drivebase", ROBOT.drivebaseLength * 0.5, ROBOT.drivebaseWidth * 0.5);
  MechanismRoot2d m_climberRoot2d = m_mech2d.getRoot("Climber", ROBOT.drivebaseLength * 0.5 + CLIMBER.kDistanceFromIntake, ROBOT.drivebaseWidth * 0.5);
  MechanismRoot2d m_shooterRoot2d = m_mech2d.getRoot("Shooter", ROBOT.drivebaseLength * 0.5 + FLYWHEEL.kDistanceFromIntake, ROBOT.drivebaseWidth * 0.5);

  MechanismLigament2d m_drivebase2d = m_drivebaseRoot2d.append(new MechanismLigament2d("Drivebase", ROBOT.drivebaseLength, 0));
  MechanismLigament2d m_limelight2d = m_drivebaseRoot2d.append(new MechanismLigament2d("Limelight", VISION.limelightHeight, 90));
  MechanismLigament2d m_intake2d = m_drivebaseRoot2d.append(new MechanismLigament2d("Intake", INTAKE.intakeLength, 0));
    MechanismLigament2d m_uptake2d = m_intake2d.append(new MechanismLigament2d("Uptake", UPTAKE.uptakeLength, 35));
  
  MechanismLigament2d m_shooter2d = m_shooterRoot2d.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(22), 90));
  MechanismLigament2d m_arm2d = m_shooter2d.append(new MechanismLigament2d("Arm", AMP.length, Units.radiansToDegrees(AMP.startingAngle + AMP.mountingAngle)));
  MechanismLigament2d m_ampShooter2d = m_arm2d.append(new MechanismLigament2d("Amp Shooter", Units.inchesToMeters(6), 0));
  
  MechanismLigament2d m_climber2d = m_climberRoot2d.append(new MechanismLigament2d("Climber", CLIMBER.kUnextendedLength, 90));
    MechanismLigament2d m_climberHook1_2d = m_climber2d.append(new MechanismLigament2d("Hook 1", Units.inchesToMeters(3), -90));
    MechanismLigament2d m_climberHook2_2d = m_climberHook1_2d.append(new MechanismLigament2d("Hook 2", Units.inchesToMeters(3), -90));
  
  public SuperStructureVisualizer(
      Intake intake,
      Uptake uptake,
      Shooter shooter,
      AmpShooter ampShooter,
      Arm arm,
      Climber climber,
      Vision vision) {
    m_intake = intake;
    m_uptake = uptake;
    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_arm = arm;
    m_climber = climber;
    m_vision = vision;
    
    m_drivebase2d.setColor(new Color8Bit(235, 137, 52));
    m_limelight2d.setColor(new Color8Bit(53, 235, 52));
    m_intake2d.setColor(new Color8Bit(235, 229, 52));
    m_uptake2d.setColor(new Color8Bit(235, 52, 176));
    m_climber2d.setColor(new Color8Bit(52, 212, 235));
    m_climberHook1_2d.setColor(new Color8Bit(52, 212, 235));
    m_climberHook2_2d.setColor(new Color8Bit(52, 212, 235));
    m_shooter2d.setColor(new Color8Bit(189, 189, 189));
    m_arm2d.setColor(new Color8Bit(235, 137, 52));
    m_ampShooter2d.setColor(new Color8Bit(235, 205, 52));
    
    SmartDashboard.putData("SuperStructure Sim", m_mech2d);
  }
  
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
