package frc.robot.networktables;

import java.util.Hashtable;

public class DeltaBoard {
  private static Hashtable<String, String> stringTable = new Hashtable<>();
  private static Hashtable<String, Double> doubleTable = new Hashtable<>();

  public static void putNumber(String key, Double val) {
    if (!doubleTable.containsKey(key) || !doubleTable.get(key).equals(val)) {
      DeltaBoard.putNumber(key, val);
      doubleTable.put(key, val);
    }
  }

  public static void putString(String key, String val) {
    if (!stringTable.containsKey(key) || !stringTable.get(key).equals(val)) {
      DeltaBoard.putString(key, val);
      stringTable.put(key, val);
    }
  }
}
