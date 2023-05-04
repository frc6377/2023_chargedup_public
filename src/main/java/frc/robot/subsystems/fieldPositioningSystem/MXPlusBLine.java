package frc.robot.subsystems.fieldPositioningSystem;

public class MXPlusBLine {
    private final double slope;
    private final double y;
    public MXPlusBLine (double slope, double y) {
        this.slope = slope;
        this.y = y;
    }
    public double getSlope() {
        return this.slope;
    }
    public double getY() {
        return this.y;
    }
}
