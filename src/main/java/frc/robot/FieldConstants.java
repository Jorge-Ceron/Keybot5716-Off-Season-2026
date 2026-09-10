package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {

  private static final Pose2d STARTING_POSE_TEST = new Pose2d(3.405, 4, new Rotation2d());
  private static final Pose2d TAXI_POSE = new Pose2d(6.0, 2.5, new Rotation2d(Math.PI));
  private static final Pose2d HUB_SHOOTING = new Pose2d(4.625, 4.037, new Rotation2d());
  private static final Pose2d RIGHT_TRENCH = new Pose2d(3.560, 0.527, new Rotation2d());

  // -- SIM
  private static final Pose2d START_POS_SIM = new Pose2d(2, 2, new Rotation2d());
  private static final Pose2d HUB_SHOOTING_SIM = new Pose2d(4.625, 4.037, new Rotation2d());

  public static boolean isRedAlliance() {
    return Robot.alliance.isPresent() && Robot.alliance.get() == DriverStation.Alliance.Red;
  }

  public static final double FIELD_LENGTH = 16.517589;
  public static final double FIELD_WIDTH = 8.049031;

  private static Pose2d toRed(Pose2d bluePose) {
    return new Pose2d(
        FIELD_LENGTH - bluePose.getX(),
        FIELD_WIDTH - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose2d allianceFlip(Pose2d pose) {
    if (isRedAlliance()) {
      return toRed(pose);
    }
    return pose;
  }

  public static Pose2d getTestingPose() {
    return allianceFlip(STARTING_POSE_TEST);
  }

  public static Pose2d getTaxiPose() {
    return allianceFlip(TAXI_POSE);
  }

  public static Pose2d getHubShootingPose() {
    return allianceFlip(HUB_SHOOTING);
  }

  public static Pose2d getRightTrenchTesting() {
    return allianceFlip(RIGHT_TRENCH);
  }

  public static Pose2d getSimulatedStartPos() {
    return allianceFlip(START_POS_SIM);
  }

  public static Pose2d getSimulatedHubShooting() {
    return allianceFlip(HUB_SHOOTING_SIM);
  }
}
