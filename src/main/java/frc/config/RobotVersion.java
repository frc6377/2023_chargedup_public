package frc.config;

import frc.robot.Constants;

public enum RobotVersion {
  V1(Constants.V1_MAC_ADDRESS),
  V2(Constants.V2_MAC_ADDRESS);

  private RobotVersion(final String rioMacAddress) {}

  public static RobotVersion getVersionForMacAddress(final String actualMacAddress) {

    System.out.println("Actual mac address" + actualMacAddress);
    switch (actualMacAddress) {
      case Constants.V2_MAC_ADDRESS:
        System.out.println(
            "Mac Adress matched V2 address (" + Constants.V2_MAC_ADDRESS + ") using V2 config");
        return V2;
      default:
        System.out.println("Selected V2 Config");
        return V2;
    }
  }
}
