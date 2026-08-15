package frc.robot.subsystems.visualizers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.util.Visualizer;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer implements Visualizer {
  private RobotState state;

  private Pose3d intakePose3d = new Pose3d();

  public RobotVisualizer(RobotState state) {
    this.state = state;
  }

  public void updateRobotVisualizer() {
    // model is chasis
    // model_0 is intake

    intakePose3d =
        new Pose3d(0.26, 0, 0.255, new Rotation3d(0, state.getArmAngle() - (Math.PI / 2), 0));

    Logger.recordOutput("ComponentsPoseArray", new Pose3d[] {intakePose3d});
  }
}
