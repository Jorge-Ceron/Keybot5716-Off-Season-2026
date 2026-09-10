package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

  class VisionIOInputs {
    public boolean ntConnected = false;

    public static class CameraInputs {
      public boolean seesTarget;

      public FiducialObservation[] fiducialAprilTagObservation;

      public int megatag1Count;
      public MegaTagPoseEstimate megatag1PoseEstimate;

      public int megatag2Count;
      public MegaTagPoseEstimate megatag2PoseEstimate;

      public int bestTagCount;
      public MegaTagPoseEstimate bestPoseEstimate;

      public Pose3d pose3d;

      public double[] standardDeviations = new double[12];
    }

    public CameraInputs cameraA = new CameraInputs();
    public CameraInputs cameraB = new CameraInputs();
  }

  void readInputs(VisionIOInputs inputs);
}
