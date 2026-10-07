package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShootCalculator {
  private RobotState robotState;
  private Rotation2d lastDriveAngle;

  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  private final LinearFilter driveAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  public static final Translation3d topCenterPoint =
      new Translation3d(4.621, FieldConstants.FIELD_WIDTH / 2.0, Units.inchesToMeters(72.0));

  public static final Translation2d nearRightCorner =
      new Translation2d(
          topCenterPoint.getX() - FieldConstants.FIELD_WIDTH / 2.0,
          FieldConstants.FIELD_WIDTH / 2.0 - FieldConstants.FIELD_WIDTH / 2.0);

  public static final Translation2d nearLeftCorner =
      new Translation2d(
          topCenterPoint.getX() - FieldConstants.FIELD_WIDTH / 2.0,
          FieldConstants.FIELD_WIDTH / 2.0 + FieldConstants.FIELD_WIDTH / 2.0);

  // Se removió hoodAngle de los parámetros de disparo
  public record shootParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double rollersSpeed,
      double distance,
      double distanceNoLook,
      double timeOfFlight,
      boolean passing) {}

  // Se simplificó el LaunchPreset al no requerir ángulo de hood
  public static record LaunchPreset(LoggedTunableNumber flywheelSpeed) {}

  private shootParameters lParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  private static double neutralZoneNear =
      FieldConstants.FIELD_LENGTH / 2.0 - Units.inchesToMeters(120);

  private static final Bounds nearHubBound =
      new Bounds(
          neutralZoneNear,
          neutralZoneNear + Units.inchesToMeters(120),
          nearRightCorner.getY(),
          nearLeftCorner.getY());

  // Passed Trench Maps (Solo velocidad y tiempo de vuelo)
  private static final InterpolatingDoubleTreeMap passedRollersSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap passedTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Launching Maps (Solo velocidad y tiempo de vuelo)
  private static final InterpolatingDoubleTreeMap rollersSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  public ShootCalculator(RobotState robotState) {
    this.robotState = robotState;
  }

  // Presets
  public static final double hubPresetDistance = 1.0;
  public static final double towerPresetDistance = 2.5;
  public static final double trenchPresetDistance = 3.03;
  public static final double outpostPresetDistance = 4.84;
  public static final double passingPresetDistance = 1.0;

  public static final LaunchPreset passingPreset;
  public static final LaunchPreset hubPreset;
  public static final LaunchPreset towerPreset;
  public static final LaunchPreset trenchPreset;
  public static final LaunchPreset outpostPreset;

  static {
    minDistance = 0.8;
    maxDistance = 5.2;
    passingMinDistance = 5.4;
    passingMaxDistance = 7.5;
    phaseDelay = 0.25;

    // Tabla de Calibración: Distancia (m) vs Velocidad Flywheel (RPM / RPS)
    rollersSpeedMap.put(2.18, 50.0);
    rollersSpeedMap.put(2.86, 54.0);
    rollersSpeedMap.put(3.72, 63.0);
    rollersSpeedMap.put(5.07, 83.0);

    // Tabla de Calibración: Distancia (m) vs Tiempo de Vuelo (s)
    timeOfFlightMap.put(2.18, 0.78);
    timeOfFlightMap.put(2.86, 1.055);
    timeOfFlightMap.put(3.72, 1.12);
    timeOfFlightMap.put(5.07, 1.725);

    // Tablas de Passing / Pase
    passedRollersSpeedMap.put(0.96, 0.0);
    passedTimeOfFlightMap.put(0.96, 0.5);

    // Inicialización de Presets limpios
    passingPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShootCalculator/Presets/Passed/RollersTestSpeed",
                rollersSpeedMap.get(passingPresetDistance)));

    hubPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShootCalculator/Presets/Hub/RollersSpeed",
                rollersSpeedMap.get(hubPresetDistance)));

    towerPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShootCalculator/Presets/Tower/RollersSpeed",
                rollersSpeedMap.get(towerPresetDistance)));

    trenchPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShootCalculator/Presets/Trench/RollersSpeed",
                rollersSpeedMap.get(trenchPresetDistance)));

    outpostPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShootCalculator/Presets/Outpost/RollersSpeed",
                rollersSpeedMap.get(outpostPresetDistance)));
  }

  public shootParameters getParameters() {
    boolean passed = robotState.passedTrench();

    Pose2d estimatedPose = robotState.getLatestFieldToRobot().getValue();
    ChassisSpeeds robotSpeeds = robotState.getLatestRobotRelativeChassisSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotSpeeds.vxMetersPerSecond * phaseDelay,
                robotSpeeds.vyMetersPerSecond * phaseDelay,
                robotSpeeds.omegaRadiansPerSecond * phaseDelay));

    Translation2d target =
        passed ? getPassingTarget() : apply(topCenterPoint.toTranslation2d(), robotState);
    Pose2d launcherPos = estimatedPose.transformBy(ShooterConstants.robotToLauncher);
    double launcherToTargetDistance = target.getDistance(launcherPos.getTranslation());

    // Calculate field relative launcher velocity
    var robotVelocity = robotState.getLatestRobotRelativeChassisSpeeds();
    var robotAngle = robotState.getLatestFieldToRobot().getValue().getRotation();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : transformVelocity(
                robotVelocity, ShooterConstants.robotToLauncher.getTranslation(), robotAngle);

    double timeOfFlight =
        passed
            ? passedTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPos;
    double lookAheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passed
              ? passedTimeOfFlightMap.get(lookAheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookAheadLauncherToTargetDistance);
      double offSetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
      double offSetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;

      lookaheadPose =
          new Pose2d(
              lookaheadPose.getTranslation().plus(new Translation2d(offSetX, offSetY)),
              lookaheadPose.getRotation());

      lookAheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }
    Pose2d lookAheadRobotPose =
        lookaheadPose.transformBy(ShooterConstants.robotToLauncher.inverse());
    Rotation2d driveAngle = target.minus(lookAheadRobotPose.getTranslation()).getAngle();

    if (lastDriveAngle == null) {
      lastDriveAngle = driveAngle;
    }
    
    double driveVelocity =
        driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians()) / 0.02;
    lastDriveAngle = driveAngle;

    var flippedPose = apply(estimatedPose, robotState);
    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());

    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub);

    double rollersSpeed =
        passed
            ? passedRollersSpeedMap.get(lookAheadLauncherToTargetDistance)
            : rollersSpeedMap.get(lookAheadLauncherToTargetDistance);

    lParameters =
        new shootParameters(
            outsideOfBadBoxes
                && lookAheadLauncherToTargetDistance >= (passed ? passingMinDistance : minDistance)
                && lookAheadLauncherToTargetDistance <= (passed ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            rollersSpeed,
            launcherToTargetDistance,
            lookAheadLauncherToTargetDistance,
            timeOfFlight,
            passed);

    Logger.recordOutput("ShootCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookAheadRobotPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookAheadLauncherToTargetDistance);

    return lParameters;
  }

  public static boolean shouldFlip(RobotState robotState) {
    return robotState.isRedAlliance();
  }

  public static double applyX(double x, RobotState robotState) {
    return shouldFlip(robotState) ? FieldConstants.FIELD_LENGTH - x : x;
  }

  public static double applyY(double y, RobotState robotState) {
    return shouldFlip(robotState) ? FieldConstants.FIELD_WIDTH - y : y;
  }

  public static Translation2d apply(Translation2d translation, RobotState robotState) {
    return new Translation2d(
        applyX(translation.getX(), robotState), applyY(translation.getY(), robotState));
  }

  public static Double apply(Double value, RobotState robotState) {
    return shouldFlip(robotState) ? -value : value;
  }

  public static Pose2d apply(Pose2d pose, RobotState robotState) {
    return shouldFlip(robotState)
        ? new Pose2d(
            apply(pose.getTranslation(), robotState), apply(pose.getRotation(), robotState))
        : pose;
  }

  public static Rotation2d apply(Rotation2d rotation, RobotState robotState) {
    return shouldFlip(robotState) ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Rotation3d apply(Rotation3d rotation, RobotState robotState) {
    return shouldFlip(robotState) ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public Translation2d getPassingTarget() {
    double flippedY = apply(robotState.getLatestFieldToRobot().getValue().getY(), robotState);
    boolean mirror = flippedY > FieldConstants.FIELD_LENGTH / 2.0;

    Translation2d flippedGoalTranslation =
        apply(
            new Translation2d(
                Units.inchesToMeters(37),
                mirror
                    ? FieldConstants.FIELD_WIDTH - Units.inchesToMeters(65)
                    : Units.inchesToMeters(65)),
            robotState);

    return flippedGoalTranslation;
  }

  public static ChassisSpeeds transformVelocity(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            - velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getSin()
                    + transform.getY() * currentRotation.getCos()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }

  public record Bounds(double minX, double maxX, double minY, double maxY) {
    public boolean contains(Translation2d translation) {
      return translation.getX() >= minX()
          && translation.getX() <= maxX()
          && translation.getY() >= minY()
          && translation.getY() <= maxY();
    }

    public Translation2d clamp(Translation2d translation) {
      return new Translation2d(
          MathUtil.clamp(translation.getX(), minX(), maxX()),
          MathUtil.clamp(translation.getY(), minY(), maxY()));
    }
  }
}