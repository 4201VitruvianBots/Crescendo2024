package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.SWERVE.MODULE;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;

public class TestCtreUtils implements AutoCloseable {
  static final double DELTA = 1e-3; // acceptable deviation range
  static final double WAIT_TIME = 0.02;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.100);
  }

  @AfterEach
  void shutdown() {
    close();
  }

  @Disabled
  public void testSensorRatioConfig() {
    /* create the TalonFX */
    var testMotor = new TalonFX(0);
    var testMotorSim = testMotor.getSimState();

    var testConfig = CtreUtils.generateTurnMotorConfig();
    testConfig.Feedback.SensorToMechanismRatio = MODULE.kTurnMotorGearRatio;
    CtreUtils.configureTalonFx(testMotor, testConfig);

    testMotorSim.setRawRotorPosition(-1);
    Timer.delay(WAIT_TIME);
    testMotor.getPosition().waitForUpdate(WAIT_TIME);

    var expectedRotation = 1.0 / MODULE.kTurnMotorGearRatio;
    var testPosition = testMotor.getPosition().getValue();

    assertEquals(expectedRotation, testPosition, DELTA);

    testMotorSim.setRawRotorPosition(1);
    Timer.delay(WAIT_TIME);
    testMotor.getPosition().waitForUpdate(WAIT_TIME);

    expectedRotation = -1.0 / MODULE.kTurnMotorGearRatio;
    testPosition = testMotor.getPosition().getValue();

    assertEquals(expectedRotation, testPosition, DELTA);
  }

  @Override
  public void close() {}
}
