package frc.robot.visualizers;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.ARM;

public class ArmVisualizer implements AutoCloseable {
  private final MechanismLigament2d m_arm2d;
  private final MechanismLigament2d m_armJoint2d;
  private final Color8Bit m_ligamentColor = new Color8Bit(235, 137, 52);
  private final String m_name;

  public ArmVisualizer(String name) {
    m_name = name;

    // Create a "line" to represent the arm.
    // We will use this to show its current position
    m_arm2d = new MechanismLigament2d(m_name, ARM.armVisualizerLength, 0);
    m_armJoint2d = new MechanismLigament2d(m_name, ARM.jointLength, 130);
    m_armJoint2d.append(m_arm2d);
  }

  public MechanismLigament2d getJointLigament() {
    return m_armJoint2d;
  }

  public MechanismLigament2d getArmLigament() {
    return m_arm2d;
  }

  public void update(double angle, double velocity) {
    m_armJoint2d.setAngle(265 - angle);
    m_arm2d.setAngle(307.5 - m_armJoint2d.getAngle() - angle);

    // Update the ligament color based on the module's current speed for easier visualization
    VisualizerUtils.updateMotorColor(m_armJoint2d, velocity, m_ligamentColor);
    VisualizerUtils.updateMotorColor(m_arm2d, velocity, m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    m_arm2d.close();
  }
}
