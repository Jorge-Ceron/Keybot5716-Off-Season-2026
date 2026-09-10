package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.DataProcessor;
import frc.robot.subsystems.vision.VisionPoseEstimateInField;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO extends DataProcessor.IODataRefresher {

  @AutoLog
  class DriveIOInputs extends SwerveDriveState {
    public double gyroAngle = 0.0;

    DriveIOInputs() {
      this.Pose = Pose2d.kZero;
    }

    public void dataSwerveState(SwerveDriveState swerveState) {
      this.Pose = swerveState.Pose;
      this.SuccessfulDaqs = swerveState.SuccessfulDaqs;
      this.FailedDaqs = swerveState.FailedDaqs;
      this.ModuleStates = swerveState.ModuleStates;
      this.ModuleTargets = swerveState.ModuleTargets;
      this.Speeds = swerveState.Speeds;
      this.OdometryPeriod = swerveState.OdometryPeriod;
    }
  }

  /**
   * Update the SwerveStates data
   *
   * @param inputs
   */
  void updateInputs(DriveIOInputs inputs);

  void getModuleStates(SwerveDriveState state);

  void resetOdometry(Pose2d pose);

  void setRequest(SwerveRequest request);

  Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

  void addVisionMeasurement(VisionPoseEstimateInField fieldEstimation);

  void setStateStandardDeviations(double x, double y, double r);

  @Override
  default void refreshData() {}
}
