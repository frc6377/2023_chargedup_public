package frc.config;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.StringJoiner;

public class IdentifyRoborio {
  /**
   * identifyRobot will attempt to find the macaddress of a robot. It does this by first looking to
   * see if it can find a ethernet address as "eth0". If no ethernet is found, then look to see if
   * usb is plugged in on "usb0". If neither of those is found, then return null.
   *
   * @return a mac address of one is found for the robot, otherwise null
   */
  public static RobotVersion identifyRobot() {
    try {
      NetworkInterface netInfo = NetworkInterface.getByName("eth0");
      if (netInfo == null) {
        netInfo = NetworkInterface.getByName("usb0");
      }
      if (netInfo == null) {
        return null;
      }
      byte[] hardwareAddress = netInfo.getHardwareAddress();
      StringJoiner sj = new StringJoiner(":");
      if (hardwareAddress == null) {
        return RobotVersion.getVersionForMacAddress("");
      }
      for (int i = 0; i < hardwareAddress.length; i++) {
        sj.add(String.format("%02X", hardwareAddress[i]));
      }
      return RobotVersion.getVersionForMacAddress(sj.toString());
    } catch (SocketException e) {
      e.printStackTrace();
      return null;
    }
  }
}
