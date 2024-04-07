package frc.robot.subsystems;

import static frc.robot.utils.TestUtils.refreshAkitData;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.utils.CtreUtils;
import org.junit.jupiter.api.BeforeEach;

public class TestRobotTime {
  static final double DELTA = 0.002; // acceptable deviation range
  static final double WAIT_TIME = 0.02;

  RobotTime m_robotTime;
  private TalonFX m_testMotor;
  private TalonFXSimState m_testMotorSimState;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    //    Logger.start();

    m_robotTime = new RobotTime();

    m_testMotor = new TalonFX(0);
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Slot0.kP = 100.0;
    motorConfigs.CustomParams.CustomParam0 = 2;
    CtreUtils.configureTalonFx(m_testMotor, motorConfigs);

    m_testMotorSimState = m_testMotor.getSimState();

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    RoboRioSim.resetData();
    refreshAkitData();

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.200);
  }
}
