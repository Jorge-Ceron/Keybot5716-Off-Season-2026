package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionPoseEstimateInField {
  private final Pose2d robotPose;
  private final double timestamp;
  private final Matrix<N3, N1> visionMeasurementStdDevs;
  private final int numTags;

  public VisionPoseEstimateInField(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs, int numTags) {
    this.robotPose = robotPose;
    this.timestamp = timestamp;
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    this.numTags = numTags;
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public double getTimestamp() {
    return timestamp;
  }

  public Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return visionMeasurementStdDevs;
  }

  public int getNumTags() {
    return numTags;
  }
}
