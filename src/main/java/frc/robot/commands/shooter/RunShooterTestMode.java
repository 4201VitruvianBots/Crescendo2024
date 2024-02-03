// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.FLYWHEEL;
import frc.robot.subsystems.Shooter;

public class RunShooterTestMode extends CommandBase {
  private final Shooter m_shooter;

  private final DoubleSubscriber kSetpointSub,
      kFSub,
      kPSub,
      kISub,
      kDSub,
      kIZoneSub,
      kGSub,
      kVSub,
      kASub;
  private double testKP, testKI, testKD, testKG, testKV, testKA;

  /** Creates a new RunShooterTestMode. */
  public RunShooterTestMode(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);

    NetworkTable shooterNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ElevatorControls");

    // initialize Test Values
    try {
      shooterNtTab.getDoubleTopic("kSetpointInches").publish().set(0);

      shooterNtTab.getDoubleTopic("kP").publish().set(FLYWHEEL.kP);
      shooterNtTab.getDoubleTopic("kI").publish().set(FLYWHEEL.kI);
      shooterNtTab.getDoubleTopic("kD").publish().set(FLYWHEEL.kD);
      shooterNtTab.getDoubleTopic("kIZone").publish().set(0);

      shooterNtTab.getDoubleTopic("kG").publish().set(FLYWHEEL.kG);
      shooterNtTab.getDoubleTopic("kV").publish().set(FLYWHEEL.kV);
      shooterNtTab.getDoubleTopic("kA").publish().set(FLYWHEEL.kA);
    } catch (Exception m_ignored) {

    }

    kSetpointSub = shooterNtTab.getDoubleTopic("kSetpointInches").subscribe(0);

    kFSub = shooterNtTab.getDoubleTopic("kF").subscribe(0);
    kPSub = shooterNtTab.getDoubleTopic("kP").subscribe(FLYWHEEL.kP);
    kISub = shooterNtTab.getDoubleTopic("kI").subscribe(FLYWHEEL.kI);
    kDSub = shooterNtTab.getDoubleTopic("kD").subscribe(FLYWHEEL.kD);
    kIZoneSub = shooterNtTab.getDoubleTopic("kIZone").subscribe(0);

    kGSub = shooterNtTab.getDoubleTopic("kG").subscribe(FLYWHEEL.kG);
    kVSub = shooterNtTab.getDoubleTopic("kV").subscribe(FLYWHEEL.kV);
    kASub = shooterNtTab.getDoubleTopic("kA").subscribe(FLYWHEEL.kA);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("USING FLYWHEEL TEST MODE!", false);
    double newSetpoint = Units.inchesToMeters(kSetpointSub.get(0));

    double newKF = kFSub.get(0);
    double newKP = kPSub.get(FLYWHEEL.kP);
    double newKI = kISub.get(FLYWHEEL.kI);
    double newKD = kDSub.get(FLYWHEEL.kD);
    double newIZone = kIZoneSub.get(0);

    double newKG = kGSub.get(FLYWHEEL.kG);
    double newKV = kVSub.get(FLYWHEEL.kV);
    double newKA = kASub.get(FLYWHEEL.kA);

    if (testKV != newKV || testKP != newKP || testKI != newKI || testKD != newKD) {
      m_shooter.setPIDvalues(newKV, newKP, newKI, newKD);
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

    m_shooter.setPercentOutput(newSetpoint);
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
