package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.DataProcessor;
import frc.robot.RobotState;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  DriveIO io;
  DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  DriveTelemetry telemetry = new DriveTelemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  RobotState robotState;

  CommandXboxController controller;

  private final Object ioLock = new Object();

  private final double maxVelocity;
  private final double maxAngularVelocity;

  private double teleopVelocityCoefficient = 1.0;
  private double rotationVelocityCoefficient = 1.0;

  private Rotation2d desiredRotationToLock = new Rotation2d();
  private Pose2d desiredPoseToAutoAllign = new Pose2d();
  private Translation2d desiredPoint = new Translation2d();

  private final ApplyRobotSpeeds pathplannerRequest =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.FieldCentricFacingAngle rotationLocked =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
  private final SwerveRequest.FieldCentricFacingAngle autoAllign =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public enum DesiredState {
    STOPPED,
    MANUAL_FIELD_DRIVE,
    ROTATION_LOCK,
    ROTATION_TO_POINT,
    DRIVE_TO_POSE
  }

  private enum SubsystemState {
    STOPPED,
    MANUAL_FIELD_DRIVE,
    ROTATED_TO_ANGLE,
    ROTATED_TO_POINT,
    DRIVEN_TO_POSE
  }

  private DesiredState desiredState = DesiredState.MANUAL_FIELD_DRIVE;
  private SubsystemState currentState = SubsystemState.MANUAL_FIELD_DRIVE;

  public DriveSubsystem(
      DriveIO io,
      RobotState robotState,
      CommandXboxController controller,
      double maxVelocity,
      double maxAngularVelocity) {
    this.io = io;
    this.robotState = robotState;
    this.controller = controller;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;

    rotationLocked.HeadingController = new PhoenixPIDController(7, 0, 0);
    rotationLocked.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    autoAllign.HeadingController = new PhoenixPIDController(5, 0, 0);
    autoAllign.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    configurePathPlanner();
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (ioLock) {
            io.updateInputs(inputs);
          }
        },
        io);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    synchronized (ioLock) {
      telemetry.telemeterize(inputs);
      Logger.processInputs("SwerveDriveInputs", inputs);
    }
    io.getModuleStates(inputs);

    robotState.incrementIteration();
    if (DriverStation.isDisabled()) {
      configureStandardDeviationsForDisabled();
    } else {
      configureStandardDeviationsForEnabled();
    }

    Logger.recordOutput("Drive/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
    Logger.recordOutput(
        "Drive/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());

    currentState = handleStateTransitions();
    Logger.recordOutput("Drive/States/Current State", currentState);
    Logger.recordOutput("Drive/States/Desired State", desiredState);
    applyStates();
  }

  private void configurePathPlanner() {
    try {
      AutoBuilder.configure(
          () -> robotState.getLatestFieldToRobot().getValue(),
          this::resetOdometry,
          () -> robotState.getLatestFusedFieldRelativeChassisSpeeds(),
          (speeds, feedforwards) ->
              setControl(
                  pathplannerRequest
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          DriveConstants.pathPlannerController,
          DriveConstants.robotConfig,
          () -> robotState.isRedAlliance(),
          this);
    } catch (Exception err) {
      DriverStation.reportError("Failed to load PathPlanner config", err.getStackTrace());
    }
  }

  private SubsystemState handleStateTransitions() {
    return switch (desiredState) {
      case MANUAL_FIELD_DRIVE -> SubsystemState.MANUAL_FIELD_DRIVE;
      case ROTATION_LOCK -> SubsystemState.ROTATED_TO_ANGLE;
      case ROTATION_TO_POINT -> SubsystemState.ROTATED_TO_POINT;
      case DRIVE_TO_POSE -> SubsystemState.DRIVEN_TO_POSE;
      case STOPPED -> SubsystemState.STOPPED;
      default -> SubsystemState.STOPPED;
    };
  }

  private void applyStates() {
    switch (currentState) {
      default:
        break;
      case MANUAL_FIELD_DRIVE:
        manualFieldDrive();
        break;
      case ROTATED_TO_ANGLE:
        rotatedToAngle();
        break;
      case ROTATED_TO_POINT:
        rotatedToPoint();
        break;
      case DRIVEN_TO_POSE:
        drivenToPose();
        break;
    }
  }

  public void setState(DesiredState state) {
    this.desiredState = state;
  }

  public void setDesiredRotationToLock(Rotation2d desiredRotation) {
    setState(DesiredState.ROTATION_LOCK);
    this.desiredRotationToLock = desiredRotation;
  }

  public void setDesiredPointToLock(Translation2d desiredPoint) {
    setState(DesiredState.ROTATION_TO_POINT);
    this.desiredPoint = desiredPoint;
  }

  public void setDesiredPoseToAutoAllign(Pose2d desiredPose) {
    setState(DesiredState.DRIVE_TO_POSE);
    this.desiredPoseToAutoAllign = desiredPose;
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  public void resetOdometry() {
    io.resetOdometry(new Pose2d());
  }

  public void setControl(SwerveRequest request) {
    io.setRequest(request);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
  }

  public void addVisionMeasurement(VisionPoseEstimateInField fieldEstimation) {
    io.addVisionMeasurement(fieldEstimation);
  }

  public void setStateStdDevs(double x, double y, double r) {
    io.setStateStandardDeviations(x, y, r);
  }

  public void configureStandardDeviationsForDisabled() {
    setStateStdDevs(1, 1, 1);
  }

  public void configureStandardDeviationsForEnabled() {
    setStateStdDevs(0.3, 0.3, 0.2);
  }

  private void manualFieldDrive() {
    var speeds = calculateSpeedsBasedOnJoystickInputs();
    io.setRequest(
        new SwerveRequest.ApplyFieldSpeeds()
            .withSpeeds(speeds)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
  }

  private void rotatedToPoint() {
    Pose2d robotPose = inputs.Pose;
    Transform2d robotToShooter = new Transform2d(new Translation2d(-0.26, 0.13), new Rotation2d());
    Pose2d shooterPose = robotPose.transformBy(robotToShooter);
    var delta = desiredPoint.minus(shooterPose.getTranslation());
    Rotation2d targetAngle = delta.getAngle();
    Rotation2d shooterFacing = new Rotation2d(Math.PI / 2);
    Rotation2d deltaAngle = targetAngle.plus(shooterFacing);

    Logger.recordOutput("Drive/DriveToPose/Traslation", delta);

    io.setRequest(
        rotationLocked
            .withVelocityX(calculateSpeedsBasedOnJoystickInputs().vxMetersPerSecond)
            .withVelocityY(calculateSpeedsBasedOnJoystickInputs().vyMetersPerSecond)
            .withTargetDirection(deltaAngle));
  }

  public boolean isAlignedToPoint() {
    var pos = inputs.Pose.getTranslation();
    var delta = desiredPoint.minus(pos);
    Rotation2d targetAngle = delta.getAngle().plus(new Rotation2d(Math.PI / 2));
    double error = Math.abs(inputs.Pose.getRotation().minus(targetAngle).getRadians());

    return error < Units.degreesToRadians(5);
  }

  public Translation2d getDesiredPoint() {
    return inputs.Pose.getTranslation();
  }

  private void rotatedToAngle() {
    var speeds = calculateSpeedsBasedOnJoystickInputs();
    io.setRequest(
        rotationLocked
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withTargetDirection(desiredRotationToLock));
  }

  public boolean isAlignedToAngle() {
    double error = Math.abs(inputs.Pose.getRotation().minus(desiredRotationToLock).getRadians());

    boolean aligned = error < Units.degreesToRadians(5);

    return aligned;
  }

  private void drivenToPose() {
    var desiredTranslation =
        desiredPoseToAutoAllign.getTranslation().minus(inputs.Pose.getTranslation());
    var linearDistance = desiredTranslation.getNorm();
    var frictionConstant = 0.0;
    if (linearDistance >= edu.wpi.first.math.util.Units.inchesToMeters(0.5)) {
      frictionConstant = DriveConstants.DRIVE_TO_POSE_STATIC_FRICTION_CONSTANT * maxVelocity;
    }
    var desiredAngle = desiredTranslation.getAngle();
    var velocityOutput = 0.0;
    if (DriverStation.isAutonomous()) {
      velocityOutput =
          Math.min(
              Math.abs(DriveConstants.autoAutoAllignController.calculate(linearDistance, 0))
                  + frictionConstant,
              DriveConstants.maxVelocityToAutoAllign);
    } else {
      velocityOutput =
          Math.min(
              Math.abs(DriveConstants.teleopAutoAllignController.calculate(linearDistance, 0))
                  + frictionConstant,
              DriveConstants.maxVelocityToAutoAllign);
    }

    var xVelocity = velocityOutput * desiredAngle.getCos();
    var yVelocity = velocityOutput * desiredAngle.getSin();

    Logger.recordOutput("Drive/DriveToPose/xVelocity", xVelocity);
    Logger.recordOutput("Drive/DriveToPose/yVelocity", yVelocity);
    Logger.recordOutput("Drive/DriveToPose/velocityOutput", velocityOutput);
    Logger.recordOutput("Drive/DriveToPose/linearDistance", linearDistance);

    io.setRequest(
        autoAllign
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withTargetDirection(desiredPoseToAutoAllign.getRotation()));
  }

  private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
    if (DriverStation.getAlliance().isEmpty()) {
      return new ChassisSpeeds(0, 0, 0);
    }

    double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), 0.1);
    double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
    double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), 0.3);

    angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

    teleopVelocityCoefficient = controller.rightTrigger().getAsBoolean() ? 0.3 : 0.75;

    double xVelocity =
        (robotState.isRedAlliance() ? xMagnitude * maxVelocity : -xMagnitude * maxVelocity)
            * teleopVelocityCoefficient;
    double yVelocity =
        (robotState.isRedAlliance() ? yMagnitude * maxVelocity : -yMagnitude * maxVelocity)
            * teleopVelocityCoefficient;
    double angularVelocity = angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

    Rotation2d skewCompensationFactor =
        Rotation2d.fromRadians(inputs.Speeds.omegaRadiansPerSecond * -0.03);

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), inputs.Pose.getRotation()),
        inputs.Pose.getRotation().plus(skewCompensationFactor));
  }

  public MapleSimSwerveDrivetrain getMapleSimDrive() {
    if (io instanceof DriveIOSim) {
      return ((DriveIOSim) io).getSimSwerve();
    }
    return null;
  }
}
