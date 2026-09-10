package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class AutoForwardTest extends AutoBuilder {

  private final PathPlannerPath test;

  public AutoForwardTest() {
    test = loadPath("TestForward");

    addCommands(Commands.deadline(new PathPlannerAuto("AutoForward")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(test);
  }

  @Override
  public Pose2d getStartingPose() {
    if (test != null) {
      return test.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
