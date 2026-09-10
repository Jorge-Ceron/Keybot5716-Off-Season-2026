package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class Auto2Test extends AutoBuilder {

  private final PathPlannerPath test3;
  private final PathPlannerPath test4;

  public Auto2Test() {
    test3 = loadPath("Testing 3");
    test4 = loadPath("Testing 4");

    addCommands(Commands.deadline(new PathPlannerAuto("TestAuto2")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(test3, test4);
  }

  @Override
  public Pose2d getStartingPose() {
    if (test3 != null) {
      return test3.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
