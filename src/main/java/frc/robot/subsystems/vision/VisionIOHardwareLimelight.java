package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import frc.lib.limelight.LimelightHelpers;
import frc.robot.RobotState;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOHardwareLimelight implements VisionIO {

  NetworkTable tableA =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightATableName);

  NetworkTable tableB =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);

  RobotState robotState;
  AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

  int imuMode = 1;

  private static final double[] DEFAULT_STDDEVS =
      new double[VisionConstants.kExpectedStdDevArrayLength];

  public VisionIOHardwareLimelight(RobotState robotState) {
    this.robotState = robotState;
    setLLSettings();
  }

  private void setLLSettings() {

    double[] cameraAPose = {
      VisionConstants.kRobotToCameraAForward,
      VisionConstants.kRobotToCameraASide,
      VisionConstants.kCameraAHeightOffGroundMeters,
      0.0,
      VisionConstants.kCameraAPitchDegrees,
      VisionConstants.kCameraAYawOffset.getDegrees()
    };

    tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);

    double[] cameraBPose = {
      VisionConstants.kRobotToCameraBForward,
      VisionConstants.kRobotToCameraBSide,
      VisionConstants.kCameraBHeightOffGroundMeters,
      0.0,
      VisionConstants.kCameraBPitchDegrees,
      VisionConstants.kCameraBYawOffset.getDegrees()
    };

    tableB.getEntry("camerapose_robotspace_set").setDoubleArray(cameraBPose);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    readCameraData(tableA, inputs.cameraA, VisionConstants.kLimelightATableName);
    readCameraData(tableB, inputs.cameraB, VisionConstants.kLimelightBTableName);
    latestInputs.set(inputs);
  }

  private void readCameraData(
      NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {

    camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;

    // 🔥 Limpia SIEMPRE para evitar datos viejos
    camera.megatag1PoseEstimate = null;
    camera.megatag2PoseEstimate = null;
    camera.bestPoseEstimate = null;
    camera.megatag1Count = 0;
    camera.megatag2Count = 0;
    camera.bestTagCount = 0;
    camera.fiducialAprilTagObservation = new FiducialObservation[0];

    if (camera.seesTarget) {
      try {

        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        var robotPose3d =
            LimelightHelpers.toPose3D(LimelightHelpers.getBotPose_wpiBlue(limelightName));

        if (mt1 != null && mt1.tagCount > 0) {
          camera.megatag1PoseEstimate = MegaTagPoseEstimate.fromLimelight(mt1);
          camera.megatag1Count = mt1.tagCount;

          camera.bestPoseEstimate = camera.megatag1PoseEstimate;
          camera.bestTagCount = camera.megatag1Count;

          camera.fiducialAprilTagObservation = FiducialObservation.fromLimelight(mt1.rawFiducials);
        }

        if (robotPose3d != null) {
          camera.pose3d = robotPose3d;
        }

        camera.standardDeviations = table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);

      } catch (Exception e) {
        System.err.println("Error processing Limelight data: " + e.getMessage());
      }
    }
  }
}
