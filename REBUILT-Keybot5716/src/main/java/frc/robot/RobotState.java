package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team254.ConcurrentTimeInterpolatableBuffer;
import frc.lib.util.State;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class RobotState implements State {
  public static final double BUFFER_TIME = 1.0;

  private final Consumer<VisionPoseEstimateInField> fieldEstimation;

  public RobotState(Consumer<VisionPoseEstimateInField> fieldEstimation) {
    this.fieldEstimation = fieldEstimation;

    fieldToRobot.addSample(0.0, Pose2d.kZero);
  }

  private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
      ConcurrentTimeInterpolatableBuffer.createBuffer(BUFFER_TIME);

  private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());

  private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);

  private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> accelX =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);
  private final ConcurrentTimeInterpolatableBuffer<Double> accelY =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(BUFFER_TIME);

  private final AtomicInteger iteration = new AtomicInteger(0);

  private double megatagTimestamp = 0;
  private Pose2d megatagPose = Pose2d.kZero;

  public void addOdometryTimeMeasurement(double timestamp, Pose2d pose) {
    fieldToRobot.addSample(timestamp, pose);
  }

  public void incrementIteration() {
    iteration.incrementAndGet();
  }

  public void addSwerveDriveMotionMeasurement(
      double timestamp,
      double angularRollRadPS,
      double angularPitchRadPS,
      double angularYawRadPS,
      double pitchRad,
      double rollRad,
      double accelX,
      double accelY,
      ChassisSpeeds desiredRobotRelativeChassisSpeeds,
      ChassisSpeeds desiredFieldRelativeChassisSpeeds,
      ChassisSpeeds measuredSpeeds,
      ChassisSpeeds measuredFieldRelativeSpeeds,
      ChassisSpeeds fusedFieldRelativeSpeeds) {
    this.driveRollAngularVelocity.addSample(timestamp, angularRollRadPS);
    this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadPS);
    this.driveYawAngularVelocity.addSample(timestamp, angularYawRadPS);
    this.drivePitchRads.addSample(timestamp, pitchRad);
    this.driveRollRads.addSample(timestamp, rollRad);
    this.accelX.addSample(timestamp, accelX);
    this.accelY.addSample(timestamp, accelY);

    this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
    this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeChassisSpeeds);
    this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
    this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
    this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
  }

  public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
    return fieldToRobot.getLatest();
  }

  public Optional<Pose2d> getFieldToRobot(double timestamp) {
    return fieldToRobot.getSample(timestamp);
  }

  public ChassisSpeeds getLatestRobotRelativeChassisSpeeds() {
    return measuredRobotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
    return measuredFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
    return desiredRobotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeeds() {
    return desiredFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeeds() {
    return fusedFieldRelativeChassisSpeeds.get();
  }

  public Pose2d getPredictedFieldToRobot(double bufferTime) {
    var fieldToRobotPredicted = getLatestFieldToRobot();
    Pose2d fieldToRobot =
        (fieldToRobotPredicted == null) ? Pose2d.kZero : fieldToRobotPredicted.getValue();
    var delta = getLatestRobotRelativeChassisSpeeds();
    delta = delta.times(bufferTime);
    return fieldToRobot.exp(
        new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
  }

  public void updateLogger() {
    if (drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/PitchAngularVel",
          drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
    }
    if (driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/RollAngularVel",
          driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
    }
    if (driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/YawAngularVel",
          driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
    }
    if (drivePitchRads.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/PitchRads", drivePitchRads.getInternalBuffer().lastEntry().getValue());
    }
    if (driveRollRads.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput(
          "RobotState/RollRads", driveRollRads.getInternalBuffer().lastEntry().getValue());
    }
    if (accelX.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput("RobotState/AccelX", accelX.getInternalBuffer().lastEntry().getValue());
    }
    if (accelY.getInternalBuffer().lastEntry() != null) {
      Logger.recordOutput("RobotState/AccelY", accelY.getInternalBuffer().lastEntry().getValue());
    }
    Logger.recordOutput(
        "RobotState/DesiredChassisSpeedRobotFrame", getLatestDesiredRobotRelativeChassisSpeeds());
    Logger.recordOutput(
        "RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeeds());
    Logger.recordOutput(
        "RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
    Logger.recordOutput(
        "RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeeds());

    Logger.recordOutput("RobotState/isRedAlliance", isRedAlliance());
    Logger.recordOutput("RobotState/PassedTrench", passedTrench());

    Logger.recordOutput("RobotState/ArmDeg", getArmDeg());
  }

  private final AtomicReference<Optional<Integer>> exclusiveTag =
      new AtomicReference<>(Optional.empty());

  public double getDrivePitchRads() {
    if (this.drivePitchRads.getInternalBuffer().lastEntry() != null) {
      return drivePitchRads.getInternalBuffer().lastEntry().getValue();
    }
    return 0.0;
  }

  public double getDriveRollRads() {
    if (this.driveRollRads.getInternalBuffer().lastEntry() != null) {
      return driveRollRads.getInternalBuffer().lastEntry().getValue();
    }
    return 0.0;
  }

  public void updateMegatagEstimate(VisionPoseEstimateInField megatag) {
    if (megatag == null) return;
    if (megatag.getRobotPose() == null) return;

    Pose2d p = megatag.getRobotPose();

    if (Double.isNaN(p.getX()) || Double.isNaN(p.getY())) return;

    megatagTimestamp = megatag.getTimestamp();
    megatagPose = p;

    fieldEstimation.accept(megatag);
  }

  public double lastMegatagTimestamp() {
    return megatagTimestamp;
  }

  public Pose2d lastMegatagPose() {
    return megatagPose;
  }

  public Optional<Integer> getExclusiveTag() {
    return exclusiveTag.get();
  }

  public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
    // Gyro yaw rate not set in sim.
    if (Robot.isReal()) return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
    return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
  }

  private Optional<Double> getMaxAbsValueInRange(
      ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
    var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
    var max = submap.stream().max(Double::compare);
    var min = submap.stream().min(Double::compare);
    if (max.isEmpty() || min.isEmpty()) return Optional.empty();
    if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
    else return min;
  }

  public static boolean passedTrench(boolean isRedAlliance, Pose2d pose) {
    return (isRedAlliance && pose.getTranslation().getX() < 11.8)
        || (!isRedAlliance && pose.getTranslation().getX() > 4.8);
  }

  public boolean passedTrench() {
    return passedTrench(this.isRedAlliance(), this.getLatestFieldToRobot().getValue());
  }

  // ----- Arm
  private final AtomicReference<Double> armAngle = new AtomicReference<>(0.0);

  public void setArmAngle(double angle) {
    armAngle.set(angle);
  }

  public double getArmAngle() {
    return armAngle.get();
  }

  // ------ Intake pivot
  private final AtomicReference<Double> armDeg = new AtomicReference<>(0.0);

  public void setIntakePivotDeg(double deg) {
    armDeg.set(deg);
  }

  public double getArmDeg() {
    return armDeg.get();
  }
}
