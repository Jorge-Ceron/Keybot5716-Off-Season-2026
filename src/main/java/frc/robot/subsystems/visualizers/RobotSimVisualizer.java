package frc.robot.subsystems.visualizers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.util.Visualizer;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import org.littletonrobotics.junction.Logger;

public class RobotSimVisualizer implements Visualizer {
  private RobotState state;

  private IntakePivotIOSim intakePivotIOSim;

  private Pose3d intakePose3d = new Pose3d();

  public RobotSimVisualizer(RobotState state, IntakePivotIOSim intakePivotIOSim) {
    this.state = state;
    this.intakePivotIOSim = intakePivotIOSim;
  }

  public void updateRobotVisualizer() {
    // model is chasis
    // model_0 is intake
    /*
    double intakeWidth = 0.5; // Example width of the intake
    double intakeHeight = 0.3; // Example height of the intake
    */
    double adentro = -90.0;

    if (intakePivotIOSim.isRunning()) {
      intakePose3d = new Pose3d(0.26, 0, 0.255, new Rotation3d(0, 0, 0));
    } else {
      intakePose3d = new Pose3d(0.26, 0, 0.255, new Rotation3d(0, adentro, 0));
    }

    // intakePose3d = new Pose3d(0.26, 0, 0.255, new Rotation3d(0, anguloRad, 0));
    // intakePose3d = new Pose3d(0.135, 0.0, 0.05, new Rotation3d(0.0,0.0,0.0));

    Logger.recordOutput("ComponentsPoseArray", new Pose3d[] {intakePose3d});
  }
}
