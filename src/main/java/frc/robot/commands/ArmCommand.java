package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmCommand extends CommandBase {
    private final ArmSubsystem subsystem;
    private final Translation2d targetPosition;
    private double lineRadius;
    private double lineThetaOffset;
    private double targetWristRotation;
    private double targetArmRotation;


    /**
     * 
     * @param targetPosition  the target arm position in cartesian coordinate
     *                            (+x is forwards, +y is vertical)
     * @param targetWristRotation the target wrist rotation from the ground
     * @param subsystem
     */
    public ArmCommand(Translation2d targetPosition, double targetWristRotation, ArmSubsystem subsystem) {
        this.subsystem = subsystem;
        this.targetPosition = targetPosition;
        this.targetWristRotation = targetWristRotation;
    }

    /**
     * Computes the value needed to for the path
     * Based on https://www.desmos.com/calculator/7w6qynhplc
     */
    private void computeInitialValues() {
        Translation2d startPosition = 
            new Translation2d(
                subsystem.currentArmExtenstion() * Math.cos(armAngle()),
                subsystem.currentArmExtenstion() * Math.sin(armAngle()));
        Translation2d delta = targetPosition.minus(startPosition);
        double angleFromStartToEnd = Math.atan2(delta.getY(),delta.getX());
        
        lineThetaOffset = angleFromStartToEnd + Math.PI/2;

        double thetaFromLineCenterToPoint = Math.atan2(startPosition.getY(), startPosition.getX()) - lineThetaOffset;
        lineRadius = startPosition.getNorm() * Math.cos(thetaFromLineCenterToPoint);
        targetArmRotation = Math.atan2(targetPosition.getY(), targetPosition.getY());
    }

    @Override
    public void initialize() {
        computeInitialValues();
    }

    @Override
    public void execute() {
        double armExtension = computeArmLength();
        double wristRotation = computeWristRotation();

        subsystem.setTarget(new ArmPosition(targetArmRotation, armExtension, wristRotation, ""));
    }

    private double computeWristRotation() {
        return targetWristRotation;
    }

    private double computeArmLength() {
        // Using (R/cos(theta + thetaOffset)) - minimumArmLength
        double denominator = Math.cos(armAngle() + lineThetaOffset);

        return Math.max((lineRadius / denominator) - Constants.ARM_LENGTH_AT_ZERO_TICKS_METERS,0);
    }

    private double armAngle() {
        return subsystem.thetaFromCANCoder();
    }
}
