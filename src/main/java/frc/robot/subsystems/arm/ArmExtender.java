package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

class ArmExtender {
  // Extend PID values
  private static final double EXTEND_PPC_KP = 0.4;
  private static final double EXTEND_PPC_KI = 0;
  private static final double EXTEND_PPC_KD = 0;
  private static final double EXTEND_PPC_MAX_V = 30;
  private static final double EXTEND_PPC_MAX_ACCEL = 30;
  private static final int EXTEND_COUNTS_PER_REV = 42;
  private static final int EXTEND_SMART_CURRENT_LIMIT = 40;

  private final CANSparkMax extendMotorLead;
  private final CANSparkMax extendMotorFollow;
  private final RelativeEncoder extendEncoder;
  private final ProfiledPIDController extendPpc;

  public ArmExtender(final int extendId1, final int extendId2) {
    extendPpc =
        new ProfiledPIDController(
            EXTEND_PPC_KP,
            EXTEND_PPC_KI,
            EXTEND_PPC_KD,
            new TrapezoidProfile.Constraints(EXTEND_PPC_MAX_V, EXTEND_PPC_MAX_ACCEL));
    extendMotorLead = new CANSparkMax(extendId1, MotorType.kBrushless);
    extendMotorLead.restoreFactoryDefaults();
    extendMotorFollow = new CANSparkMax(extendId2, MotorType.kBrushless);
    extendMotorFollow.restoreFactoryDefaults();
    extendMotorFollow.follow(extendMotorLead);
    extendEncoder =
        extendMotorLead.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, EXTEND_COUNTS_PER_REV);

    extendMotorLead.setSmartCurrentLimit(EXTEND_SMART_CURRENT_LIMIT);
  }

  public void periodic() {
    extendMotorLead.set(extendPpc.calculate(extendEncoder.getPosition()));
  }

  public void setLength(double position) {
    // controller.setReference(position, CANSparkMax.ControlType.kPosition);
    extendPpc.setGoal(position);
  }

  public double getLength() {
    return extendEncoder.getPosition() * Constants.armLengthTicksToMeters
        + Constants.armLengthAtZeroTicks;
  }
}
