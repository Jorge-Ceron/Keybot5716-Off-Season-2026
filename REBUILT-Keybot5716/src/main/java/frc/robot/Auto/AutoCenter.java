package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class AutoCenter extends AutoBuilder {
  private final PathPlannerPath center_to_score;

  public AutoCenter() {
    center_to_score = loadPath("CENTER_TO_SCORE");

    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("CENTER_AUTO"))));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(center_to_score);
  }

  @Override
  public Pose2d getStartingPose() {
    if (center_to_score != null) {
      return center_to_score.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
