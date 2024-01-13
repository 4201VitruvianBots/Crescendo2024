// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CLIMBER;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  
  // Mechanism2d visualization setup
  public Mechanism2d mech2d = new Mechanism2d(CLIMBER.kMaxHeightMeters, CLIMBER.kMaxHeightMeters);
  public MechanismRoot2d root2d = mech2d.getRoot("Climber", CLIMBER.kMaxHeightMeters / 2, 0);
  public MechanismLigament2d climberLigament2d =
      root2d.append(new MechanismLigament2d("Climber", CLIMBER.kMaxHeightMeters, 90));
  public MechanismLigament2d hook1Ligament2d = climberLigament2d.append(new MechanismLigament2d("Hook 1", 0.1, 0));
  public MechanismLigament2d hook2Ligament2d = hook1Ligament2d.append(new MechanismLigament2d("Hook 2", 0.1, -90));
  
  public Climber() {
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Climber Sim", mech2d);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
