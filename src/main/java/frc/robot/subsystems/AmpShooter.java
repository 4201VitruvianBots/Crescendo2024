package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.CAN;
import frc.robot.constants.ROBOT;
import frc.robot.utils.CtreUtils;
import org.littletonrobotics.junction.Logger;

public class AmpShooter extends SubsystemBase {
  private Intake m_intake;
  private final TalonFX m_ampMotor = new TalonFX(CAN.ampShooter);
  private double m_ampAutoPercentOutput;

  private final DCMotorSim m_ampMotorSim =
      new DCMotorSim(AMPSHOOTER.AmpGearbox, AMPSHOOTER.gearRatio, AMPSHOOTER.Inertia);

  private final TalonFXSimState m_ampMotorSimState = m_ampMotor.getSimState();

  public AmpShooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = AMPSHOOTER.kP;
    config.Slot0.kI = AMPSHOOTER.kI;
    config.Slot0.kD = AMPSHOOTER.kD;
    config.Feedback.SensorToMechanismRatio = AMPSHOOTER.gearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CtreUtils.configureTalonFx(m_ampMotor, config);
  }

  public void registerIntake(Intake intake) {
    m_intake = intake;
  }

  public void setPercentOutput(double percentOutput) {
    if (DriverStation.isAutonomous()) {
        m_ampAutoPercentOutput = percentOutput;
    } else {
        m_ampAutoPercentOutput = 0;
        m_ampMotor.set(percentOutput);
    }
  }
  
  public double getSpeed() {
    return m_ampMotor.get();
  }

  public double getRpm() {
    return m_ampMotor.getVelocity().getValue() * 60.0;
  }

  private void updateLogger() {
    Logger.recordOutput("AmpShooter/Velocity", m_ampMotor.getVelocity().getValue());
    Logger.recordOutput("AmpShooter/Percentage", m_ampMotor.getMotorVoltage().getValue() / 12.0);
    Logger.recordOutput("AmpShooter/Current", m_ampMotor.getTorqueCurrent().getValue());
  }

  @Override
  public void periodic() {
    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();

    if (m_intake != null) {
      if (DriverStation.isAutonomous() && m_ampAutoPercentOutput != 0) {
        if (m_intake.getSensorInput1() || m_intake.getSensorInput2()) {
          setPercentOutput(0);
        } else {
          setPercentOutput(m_ampAutoPercentOutput);
        }
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    m_ampMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_ampMotorSim.setInputVoltage(MathUtil.clamp(m_ampMotorSimState.getMotorVoltage(), -12, 12));

    m_ampMotorSim.update(RobotTime.getTimeDelta());

    m_ampMotorSimState.setRawRotorPosition(
        m_ampMotorSim.getAngularPositionRotations() * AMPSHOOTER.gearRatio);
    m_ampMotorSimState.setRotorVelocity(
        m_ampMotorSim.getAngularVelocityRPM() * AMPSHOOTER.gearRatio / 60.0);
  }
}
