// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SHOOTER;
import frc.robot.subsystems.Shooter;

public class RunShooterTestMode extends Command {
  private final Shooter m_shooter;

  private final DoubleSubscriber kSetpointSub, kFSub, kPSub, kISub, kDSub, kGSub, kVSub, kASub;
  private double testKP, testKI, testKD, testKG, testKV, testKA;

  /** Creates a new RunShooterTestMode. */
  public RunShooterTestMode(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);

    NetworkTable shooterNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ShooterControls");

    // initialize Test Values
    try {
      shooterNtTab.getDoubleTopic("SetpointRPM").publish().set(0);

      shooterNtTab.getDoubleTopic("kP").publish().set(SHOOTER.kPTop);
      shooterNtTab.getDoubleTopic("kI").publish().set(SHOOTER.kITop);
      shooterNtTab.getDoubleTopic("kD").publish().set(SHOOTER.kDTop);

      shooterNtTab.getDoubleTopic("kG").publish().set(SHOOTER.kS);
      shooterNtTab.getDoubleTopic("kV").publish().set(SHOOTER.kV);
      shooterNtTab.getDoubleTopic("kA").publish().set(SHOOTER.kA);
    } catch (Exception m_ignored) {

    }

    kSetpointSub = shooterNtTab.getDoubleTopic("SetpointRPM").subscribe(0);

    kFSub = shooterNtTab.getDoubleTopic("kF").subscribe(0);
    kPSub = shooterNtTab.getDoubleTopic("kP").subscribe(SHOOTER.kPTop);
    kISub = shooterNtTab.getDoubleTopic("kI").subscribe(SHOOTER.kITop);
    kDSub = shooterNtTab.getDoubleTopic("kD").subscribe(SHOOTER.kDTop);

    kGSub = shooterNtTab.getDoubleTopic("kG").subscribe(SHOOTER.kS);
    kVSub = shooterNtTab.getDoubleTopic("kV").subscribe(SHOOTER.kV);
    kASub = shooterNtTab.getDoubleTopic("kA").subscribe(SHOOTER.kA);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportWarning("USING SHOOTER TEST MODE!", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double newSetpoint = (kSetpointSub.get(0));

    double newKF = kFSub.get(0);
    double newKP = kPSub.get(SHOOTER.kPTop);
    double newKI = kISub.get(SHOOTER.kITop);
    double newKD = kDSub.get(SHOOTER.kDTop);

    double newKG = kGSub.get(SHOOTER.kS);
    double newKV = kVSub.get(SHOOTER.kV);
    double newKA = kASub.get(SHOOTER.kA);

    if (testKV != newKV || testKP != newKP || testKI != newKI || testKD != newKD) {
      m_shooter.setPidValues(newKV, newKP, newKI, newKD);
      testKV = newKV;
      testKP = newKP;
      testKI = newKI;
      testKD = newKD;
    }

    if (testKG != newKG || testKV != newKV || testKA != newKA) {
      m_shooter.setSimpleMotorFeedForward(newKG, newKV, newKA);
      testKG = newKG;
      testKV = newKV;
      testKA = newKA;
    }

    m_shooter.setRPMOutput(newSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
