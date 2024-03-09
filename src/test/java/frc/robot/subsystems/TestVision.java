package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestVision {

  @BeforeEach
  public void constructDevices() {}

  @AfterEach
  void shutdown() throws Exception {}

  @Test
  public void testRobotToTargetAngle() {
    var robotPose = new Translation2d(0, 0);
    var testTarget = new Translation2d(1, 1);

    var targetAngle = robotPose.plus(testTarget).getAngle();

    assertEquals(Rotation2d.fromDegrees(45), targetAngle);
  }

  @Test
  public void testModuleConstants() {}
}
