package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MXPlusBLine {
  private final double slope;
  private final double y;
  private final Translation2d tagPos;

  public MXPlusBLine(double slope, double y, Translation2d tagPos) {
    this.slope = slope;
    this.y = y;
    this.tagPos = tagPos;
  }

  public double getSlope() {
    return this.slope;
  }

  public double getY() {
    return this.y;
  }

  public Translation2d getTagPose() {
    return this.tagPos;
  }

  public Translation2d getYIntersect() {
    return new Translation2d(0, this.y);
  }

  public Translation2d getIntersection(MXPlusBLine otherLine) {
    double intersectionX = (otherLine.getY() - this.y) / (this.slope - otherLine.getSlope());
    double intersectionY = intersectionX * this.slope + this.y;
    return new Translation2d(intersectionX, intersectionY);
  }

  public Translation2d getNearestPointOn(Translation2d comparisonPoint) {
    if (this.slope == 0) {
      return new Translation2d(comparisonPoint.getX(), y);
    }
    double comparisonLineSlope = -1 / this.slope;
    double comparisonLineY = comparisonPoint.getY() - comparisonLineSlope * comparisonPoint.getX();
    return getIntersection(new MXPlusBLine(comparisonLineSlope, comparisonLineY, this.tagPos));
  }
}
