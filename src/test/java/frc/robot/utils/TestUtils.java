package frc.robot.utils;

import java.lang.reflect.Field;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;

public class TestUtils {
  public static void setPrivateField(Object instance, String fieldName, Object valueToSet) {
    try {
      Field f = instance.getClass().getDeclaredField(fieldName);
      f.setAccessible(true);
      f.set(instance, valueToSet);
    } catch (Exception e) {
      System.out.println(
          "Could not set field '" + fieldName + "' in Object '" + instance.toString() + "'");
    }
  }

  public static Object getPrivateObject(Object instance, String fieldName) {
    try {
      Field f = instance.getClass().getDeclaredField(fieldName);
      f.setAccessible(true);
      return f.get(instance);
    } catch (Exception e) {
      System.out.println(
          "Could not get field '" + fieldName + "' in Object '" + instance.toString() + "'");
      return null;
    }
  }

  public static void refreshAkitData() {
    ConduitApi.getInstance().captureData();
    LoggedDriverStation.periodic();
    LoggedSystemStats.periodic();
  }
}
