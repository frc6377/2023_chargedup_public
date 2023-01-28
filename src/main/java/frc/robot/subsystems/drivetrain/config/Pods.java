package frc.robot.subsystems.drivetrain.config;

import frc.robot.subsystems.drivetrain.config.PodConstants.PodConstantsBuilder;
import java.util.HashMap;

public class Pods {
  private static HashMap<PodName, PodConstantsBuilder> configs = new HashMap<>();
  private static boolean intited = false;

  private static void init() {
    intited = true;
    Pods.addPod(
        PodName.A,
        new PodConstantsBuilder().setEncoderID(1).setEncoderOffset(-Math.toRadians(55.0)));

    Pods.addPod(
        PodName.B,
        new PodConstantsBuilder().setEncoderID(3).setEncoderOffset(-Math.toRadians(108.0)));

    Pods.addPod(
        PodName.C,
        new PodConstantsBuilder().setEncoderID(5).setEncoderOffset(Math.toRadians(189.0)));

    Pods.addPod(
        PodName.D,
        new PodConstantsBuilder().setEncoderID(7).setEncoderOffset(-Math.toRadians(196.0)));

    // Unconfigured, please test and add correct values
    Pods.addPod(
        PodName.F,
        new PodConstantsBuilder().setEncoderID(11).setEncoderOffset(Math.toRadians(0)));
    
    Pods.addPod(
        PodName.G,
        new PodConstantsBuilder().setEncoderID(13).setEncoderOffset(Math.toRadians(0)));

    Pods.addPod(
        PodName.H,
        new PodConstantsBuilder().setEncoderID(15).setEncoderOffset(Math.toRadians(0)));

    Pods.addPod(
        PodName.I,
        new PodConstantsBuilder().setEncoderID(17).setEncoderOffset(Math.toRadians(0)));
  }

  public static void addPod(PodName name, PodConstantsBuilder constants) {
    configs.put(name, constants);
  }

  public static PodConstants getPod(PodName name, PodLocation location) {
    if (!intited) init();
    return configs.get(name).setLocation(location).build();
  }
}
