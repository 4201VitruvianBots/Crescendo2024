package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.CAN;
import frc.robot.constants.SWERVE;
import frc.robot.utils.ModuleMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestVision {

  @BeforeEach
  public void constructDevices() {}

  @AfterEach
  void shutdown() throws Exception {}

  @Test
  public void testRobotToTargetAngle() {
    var robotPose = new Translation2d(0,0);
    var testTarget = new Translation2d(1, 1);

    var targetAngle = robotPose.plus(testTarget).getAngle();

    assertEquals(Rotation2d.fromDegrees(45), targetAngle);
  }

  @Test
  public void testModuleConstants() {
  }
}
