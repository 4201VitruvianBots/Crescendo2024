package frc.robot;

import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestSim {

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    refreshAkitData();
  }

  @Test
  public void testBattery() {
    assertEquals(12, BatterySim.calculateDefaultBatteryLoadedVoltage(0));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(0));
    assertEquals(12, RobotController.getBatteryVoltage());
  }
}
