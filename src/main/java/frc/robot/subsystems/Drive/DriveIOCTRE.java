package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {

  AtomicReference<SwerveDriveState> telemetryCache = new AtomicReference<>();

  private final StatusSignal<AngularVelocity> angularPitchVelocity;
  private final StatusSignal<AngularVelocity> angularRollVelocity;
  private final StatusSignal<AngularVelocity> angularYawVelocity;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;

  private RobotState robotState;

  public DriveIOCTRE(
      RobotState robotState,
      SwerveDrivetrainConstants swerveConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, swerveConstants, 250, modules);
    this.robotState = robotState;

    angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
    angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
    angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
    roll = getPigeon2().getRoll();
    pitch = getPigeon2().getPitch();
    accelerationX = getPigeon2().getAccelerationX();
    accelerationY = getPigeon2().getAccelerationY();

    BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, angularPitchVelocity, angularRollVelocity, roll, pitch, accelerationX, accelerationY);

    this.getOdometryThread().setThreadPriority(99);

    registerTelemetry(telemetryConsumer);
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void setRequest(SwerveRequest request) {
    super.setControl(request);
  }

  @Override
  public Command applyRequest(
      Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
    return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
  }

  @Override
  public void addVisionMeasurement(VisionPoseEstimateInField fieldEstimate) {
    if (fieldEstimate.getVisionMeasurementStdDevs() == null) {
      this.addVisionMeasurement(
          fieldEstimate.getRobotPose(), Utils.fpgaToCurrentTime(fieldEstimate.getTimestamp()));
    } else {
      this.addVisionMeasurement(
          fieldEstimate.getRobotPose(),
          Utils.fpgaToCurrentTime(fieldEstimate.getTimestamp()),
          fieldEstimate.getVisionMeasurementStdDevs());
    }
  }

  @Override
  public void setStateStandardDeviations(double x, double y, double r) {
    Matrix<N3, N1> stateStdDevs = VecBuilder.fill(x, y, r);
    this.setStateStdDevs(stateStdDevs);
  }

  Consumer<SwerveDriveState> telemetryConsumer =
      state -> {
        telemetryCache.set(state.clone());
        robotState.addOdometryTimeMeasurement(
            Timer.getFPGATimestamp() / Utils.getCurrentTimeSeconds() + state.Timestamp, state.Pose);
      };

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    var state = telemetryCache.get();
    if (state == null) return;
    inputs.dataSwerveState(state);
    var gyroRotation = inputs.Pose.getRotation();
    inputs.gyroAngle = gyroRotation.getDegrees();
    var measuredRobotRelativeChassisSpeeds = getKinematics().toChassisSpeeds(inputs.ModuleStates);
    var measuredFieldRelativeChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRelativeChassisSpeeds, gyroRotation);
    var desiredRobotRelativeChassisSpeeds = getKinematics().toChassisSpeeds(inputs.ModuleTargets);
    var desiredFieldRelativeChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(desiredRobotRelativeChassisSpeeds, gyroRotation);

    double timestamp = Timer.getFPGATimestamp();
    double rollRadsPs = Units.degreesToRadians(angularRollVelocity.getValueAsDouble());
    double pitchRadsPS = Units.degreesToRadians(angularPitchVelocity.getValueAsDouble());
    double yawRadPS = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());

    var fusedFieldRelativeChassisSpeeds =
        new ChassisSpeeds(
            measuredFieldRelativeChassisSpeeds.vxMetersPerSecond,
            measuredFieldRelativeChassisSpeeds.vyMetersPerSecond,
            yawRadPS);

    double pitchRads = Units.degreesToRadians(pitch.getValueAsDouble());
    double rollRads = Units.degreesToRadians(roll.getValueAsDouble());
    double accelX = accelerationX.getValueAsDouble();
    double accelY = accelerationY.getValueAsDouble();

    robotState.addSwerveDriveMotionMeasurement(
        timestamp,
        rollRadsPs,
        pitchRadsPS,
        yawRadPS,
        pitchRads,
        rollRads,
        accelX,
        accelY,
        desiredRobotRelativeChassisSpeeds,
        desiredFieldRelativeChassisSpeeds,
        measuredRobotRelativeChassisSpeeds,
        measuredFieldRelativeChassisSpeeds,
        fusedFieldRelativeChassisSpeeds);
  }

  @Override
  public void getModuleStates(SwerveDriveState driveState) {
    final String[] moduleNames = {"Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR"};
    if (driveState.ModuleStates == null) return;

    for (int i = 0; i < getModules().length; i++) {
      Logger.recordOutput(
          moduleNames[i] + " Absolute Encoder Angle",
          getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
      Logger.recordOutput(moduleNames[i] + " Steering Angle", driveState.ModuleStates[i].angle);
      Logger.recordOutput(
          moduleNames[i] + " Target Steering Angle", driveState.ModuleTargets[i].angle);
      Logger.recordOutput(
          moduleNames[i] + " Drive Velocity", driveState.ModuleStates[i].speedMetersPerSecond);
      Logger.recordOutput(
          moduleNames[i] + " Target Drive Velocity",
          driveState.ModuleTargets[i].speedMetersPerSecond);
    }
  }

  @Override
  public void refreshData() {
    BaseStatusSignal.refreshAll(
        angularRollVelocity,
        angularPitchVelocity,
        angularYawVelocity,
        pitch,
        roll,
        accelerationX,
        accelerationY);
  }
}
