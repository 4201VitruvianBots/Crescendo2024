package frc.robot.subsystems;

import static frc.robot.utils.TestUtils.refreshAkitData;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.CAN;
import frc.robot.constants.SWERVE;
import frc.robot.utils.ModuleMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class TestSwerveDrive implements AutoCloseable {
  static final double DELTA = 0.2; // acceptable deviation range
  static final double WAIT_TIME = 0.02;

  NetworkTableInstance m_nt;
  NetworkTable m_table;

  RobotTime m_robotTime;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    Logger.start();

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    RoboRioSim.resetData();
    refreshAkitData();

    m_nt = NetworkTableInstance.getDefault();
    m_nt.setServer("localhost", NetworkTableInstance.kDefaultPort4 + 1);
    m_nt.startClient4("unittest");
    m_table = m_nt.getTable("unittest");

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.200);
  }

  @AfterEach
  void shutdown() throws Exception {
    close();
  }

  @Test
  public void testModuleKinematics() {
    for (ModuleMap.MODULE_POSITION i : ModuleMap.MODULE_POSITION.values()) {
      var xPos = SWERVE.DRIVE.kModuleTranslations.get(i).getX();
      var yPos = SWERVE.DRIVE.kModuleTranslations.get(i).getY();

      switch (i) {
        case FRONT_LEFT -> assertTrue(xPos > 0 && yPos > 0);
        case FRONT_RIGHT -> assertTrue(xPos > 0 && yPos < 0);
        case BACK_LEFT -> assertTrue(xPos < 0 && yPos > 0);
        case BACK_RIGHT -> assertTrue(xPos < 0 && yPos < 0);
      }
    }
  }

  @Test
  public void testModuleConstants() {
    for (ModuleMap.MODULE_POSITION i : ModuleMap.MODULE_POSITION.values()) {
      var moduleConstants = SWERVE.MODULE_CONSTANTS[i.ordinal()];

      switch (i) {
        case FRONT_LEFT -> {
          assertEquals(moduleConstants.DriveMotorId, CAN.frontLeftDriveMotor);
          assertEquals(moduleConstants.SteerMotorId, CAN.frontLeftTurnMotor);
          assertEquals(moduleConstants.CANcoderId, CAN.frontLeftCanCoder);
          assertEquals(moduleConstants.CANcoderOffset, SWERVE.DRIVE.kFrontLeftEncoderOffset);
        }
        case FRONT_RIGHT -> {
          assertEquals(moduleConstants.DriveMotorId, CAN.frontRightDriveMotor);
          assertEquals(moduleConstants.SteerMotorId, CAN.frontRightTurnMotor);
          assertEquals(moduleConstants.CANcoderId, CAN.frontRightCanCoder);
          assertEquals(moduleConstants.CANcoderOffset, SWERVE.DRIVE.kFrontRightEncoderOffset);
        }
        case BACK_LEFT -> {
          assertEquals(moduleConstants.DriveMotorId, CAN.backLeftDriveMotor);
          assertEquals(moduleConstants.SteerMotorId, CAN.backLeftTurnMotor);
          assertEquals(moduleConstants.CANcoderId, CAN.backLeftCanCoder);
          assertEquals(moduleConstants.CANcoderOffset, SWERVE.DRIVE.kBackLeftEncoderOffset);
        }
        case BACK_RIGHT -> {
          assertEquals(moduleConstants.DriveMotorId, CAN.backRightDriveMotor);
          assertEquals(moduleConstants.SteerMotorId, CAN.backRightTurnMotor);
          assertEquals(moduleConstants.CANcoderId, CAN.backRightCanCoder);
          assertEquals(moduleConstants.CANcoderOffset, SWERVE.DRIVE.kBackRightEncoderOffset);
        }
      }
      assertEquals(moduleConstants.LocationX, SWERVE.DRIVE.kModuleTranslations.get(i).getX());
      assertEquals(moduleConstants.LocationY, SWERVE.DRIVE.kModuleTranslations.get(i).getY());
    }
  }

  @Override
  public void close() throws Exception {
    /* destroy our TalonFX object */
    //    m_testModule.close();
  }
}
