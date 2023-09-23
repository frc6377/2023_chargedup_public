package frc.robot.networktables;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Hashtable;

public class DeltaBoard {
  private static Hashtable<String, String> stringTable = new Hashtable<>();
  private static Hashtable<String, Double> doubleTable = new Hashtable<>();
  private static Hashtable<String, Boolean> booleanTable = new Hashtable<>();

  public static void putNumber(String key, Double val) {
    if (!doubleTable.containsKey(key) || !doubleTable.get(key).equals(val)) {
      SmartDashboard.putNumber(key, val);
      doubleTable.put(key, val);
    }
  }

  public static void putString(String key, String val) {
    if (!stringTable.containsKey(key) || !stringTable.get(key).equals(val)) {
      SmartDashboard.putString(key, val);
      stringTable.put(key, val);
    }
  }

  public static void putBoolean(String key, boolean val) {
    if (!stringTable.containsKey(key) || !stringTable.get(key).equals(val)) {
      SmartDashboard.putBoolean(key, val);
      booleanTable.put(key, val);
    }
  }
}
