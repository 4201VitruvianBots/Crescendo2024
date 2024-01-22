package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotTime extends SubsystemBase {
  private static double m_time;
  private static double m_lastTime;
  private static double m_dt;
  private static TIME_MODE m_timeMode = TIME_MODE.FPGA_TIME;

  enum TIME_MODE {
    FPGA_TIME,
    CTRE_TIME,
    TIMESTEP
  }

  public RobotTime() {
    if (RobotBase.isSimulation()) m_timeMode = TIME_MODE.CTRE_TIME;
    else m_timeMode = TIME_MODE.FPGA_TIME;
  }

  public static double getTime() {
    return m_time;
  }

  public static double getTimeDelta() {
    return m_dt;
  }

  public static TIME_MODE getTimeMode() {
    return m_timeMode;
  }

  public static void setTime(double time) {
    m_time = time;
  }

  public static void setTimeMode(TIME_MODE timeMode) {
    m_timeMode = timeMode;
  }

  @Override
  public void periodic() {
    switch (m_timeMode) {
      case TIMESTEP:
        if (RobotBase.isSimulation()) {
          setTime(getTime() + 0.02);
          break;
        }
      case CTRE_TIME:
        setTime(Utils.getCurrentTimeSeconds());
        break;
      case FPGA_TIME:
      default:
        setTime(Timer.getFPGATimestamp());
    }
    m_dt = getTime() - m_lastTime;

    m_lastTime = getTime();
  }
}
