package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.utils.ModuleMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestSwerveConstants {

  @BeforeEach
  public void constructDevices() {}

  @AfterEach
  void shutdown() throws Exception {}

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
      assertEquals(moduleConstants.DriveMotorInverted, SWERVE.DRIVE.kDriveInversions[i.ordinal()]);
      assertEquals(moduleConstants.SteerMotorInverted, SWERVE.DRIVE.kTurnInversions[i.ordinal()]);
      assertEquals(moduleConstants.LocationX, SWERVE.DRIVE.kModuleTranslations.get(i).getX());
      assertEquals(moduleConstants.LocationY, SWERVE.DRIVE.kModuleTranslations.get(i).getY());
    }
  }
}
