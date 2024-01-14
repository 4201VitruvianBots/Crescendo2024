package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;

public class AmpShooter extends SubsystemBase {
  private final CANSparkMax ampMotor = new CANSparkMax(CAN.ampShooter, MotorType.kBrushless);
  private double ampMaxRPM = 1;
  private double ampShooterRPM;

  public AmpShooter() {}

  public void maxRPM() {
    ampMotor.set(ampMaxRPM);
    ampShooterRPM = ampMaxRPM;
  }

  public void minRPM() {
    ampMotor.set(0);
    ampShooterRPM = 0;
  }

  public double getRPM() {
    return ampShooterRPM;
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("ampShooterRPM", this.getRPM());
  }

  @Override
  public void periodic() {
    this.updateShuffleboard();
  }
}
