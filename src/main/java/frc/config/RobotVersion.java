package frc.config;

import frc.robot.Constants;

public enum RobotVersion {
  V1(Constants.V1_MAC_ADDRESS),
  V2(Constants.V2_MAC_ADDRESS);

  private RobotVersion(final String rioMacAddress) {}

  public static RobotVersion getVersionForMacAddress(final String actualMacAddress) {

    System.out.println("Actual mac address" + actualMacAddress);
    switch (actualMacAddress) {
      case Constants.V1_MAC_ADDRESS:
        System.out.println(
            "Mac Address matched V1 address (" + Constants.V1_MAC_ADDRESS + ") using V1 config.");
        return V1;
      default:
        System.out.println("Selected V2 Config");
        return V2;
    }
  }
}
