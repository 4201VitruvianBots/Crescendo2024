package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import org.littletonrobotics.junction.Logger;

public class AmpShooter extends SubsystemBase {
  private final CANSparkMax ampMotor = new CANSparkMax(CAN.ampShooter, MotorType.kBrushless);
  private final SparkMaxPIDController pidController = ampMotor.getPIDController();
  private final RelativeEncoder encoder = ampMotor.getEncoder();
  private double m_speed;

  public AmpShooter() {
    ampMotor.restoreFactoryDefaults();
    encoder.setVelocityConversionFactor(0);
    pidController.setFeedbackDevice(encoder);
    pidController.setP(0);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setOutputRange(0, 0);
  }

  public void setPercentOutput(Double m_speed) {
    pidController.setReference(m_speed, CANSparkMax.ControlType.kVelocity);
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  private void updateShuffleboard() {
    //    SmartDashboard.putNumber("ampShooterRPM", this.getRPM());
  }

  private void updateLog() {
    Logger.recordOutput("AmpShooter/RPM", getRPM());
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    updateLog();
  }
}
