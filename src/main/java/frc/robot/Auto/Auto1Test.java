package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class Auto1Test extends AutoBuilder {

  private final PathPlannerPath test;
  private final PathPlannerPath test2;

  public Auto1Test() {
    test = loadPath("Testing");
    test2 = loadPath("Testing2");

    addCommands(Commands.deadline(new PathPlannerAuto("TestAuto")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(test, test2);
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
