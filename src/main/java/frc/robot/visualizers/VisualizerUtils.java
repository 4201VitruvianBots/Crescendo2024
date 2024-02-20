package frc.robot.visualizers;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;

public class VisualizerUtils {
  static class MechanismDisplay implements AutoCloseable {
    private final Mechanism2d m_display = new Mechanism2d(1, 1);
    private final ArrayList<MechanismRoot2d> m_roots = new ArrayList<>();
    private final String m_name;

    public MechanismDisplay(double x, double y, MechanismLigament2d mechanismLigament2d) {
      m_name = mechanismLigament2d.getName();
      addLigament(x, y, mechanismLigament2d);
    }

    public void addLigament(double x, double y, MechanismLigament2d mechanismLigament2d) {
      var m_root = m_display.getRoot(mechanismLigament2d.getName() + "_root", x, y);
      m_root.append(mechanismLigament2d);

      m_roots.add(m_root);
    }

    public void addSmartDashboardDisplay() {
      SmartDashboard.putData(m_name, m_display);
    }

    @Override
    public void close() throws Exception {
      for (var root : m_roots) root.close();
      m_display.close();
    }
  }

  /* Function to visualize the speed of a particular motor. */
  public static void updateMotorColor(
      MechanismLigament2d ligament, double motorSpeed, Color8Bit originalColor) {
    double deltaBrightness = Math.abs(motorSpeed) * 75;

    Color8Bit newColor =
        new Color8Bit(
            originalColor.red + (int) deltaBrightness,
            originalColor.green + (int) deltaBrightness,
            originalColor.blue + (int) deltaBrightness);

    ligament.setColor(newColor);
  }
}
