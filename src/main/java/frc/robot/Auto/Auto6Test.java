package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class Auto6Test extends AutoBuilder {

  private final PathPlannerPath test14;
  private final PathPlannerPath test15;
  private final PathPlannerPath test16;

  public Auto6Test() {

    test14 = loadPath("Testing14");
    test15 = loadPath("Testing15");
    test16 = loadPath("Testing16");

    addCommands(Commands.deadline(new PathPlannerAuto("TestAuto")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(test14, test15, test16);
  }

  @Override
  public Pose2d getStartingPose() {
    if (test14 != null) {
      return test14.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
