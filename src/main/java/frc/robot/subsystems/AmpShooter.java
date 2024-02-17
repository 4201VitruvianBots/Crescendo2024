package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AMP;
import frc.robot.constants.CAN;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class AmpShooter extends SubsystemBase {
  private final TalonFX ampMotor = new TalonFX(CAN.ampShooter);

  public AmpShooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = AMP.kP;
    config.Slot0.kI = AMP.kI;
    config.Slot0.kD = AMP.kD;
    config.Feedback.SensorToMechanismRatio = AMP.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(ampMotor, config);
  }

  public void setPercentOutput(double percentOutput) {
    ampMotor.set(percentOutput);
  }

  public double getSpeed() {
    return ampMotor.get();
  }

  private void updateLogger() {
    Logger.recordOutput("AmpShooter/Velocity", ampMotor.getVelocity().getValue());
    Logger.recordOutput("AmpShooter/Percentage", ampMotor.get());
  }

  @Override
  public void periodic() {
    if (!ROBOT.disableLogging) updateLogger();
  }
}
