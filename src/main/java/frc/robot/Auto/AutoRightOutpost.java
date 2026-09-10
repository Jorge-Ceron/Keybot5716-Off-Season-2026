package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class AutoRightOutpost extends AutoBuilder {
  private final PathPlannerPath right_to_outpost;
  private final PathPlannerPath outpost_to_hub;

  public AutoRightOutpost() {
    right_to_outpost = loadPath("RIGHT_START_TO_OUTPOST");
    outpost_to_hub = loadPath("OUTPOST_TO_HUB");

    addCommands(Commands.deadline(new PathPlannerAuto("RIGHT_OUTPOST_AUTO")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(right_to_outpost, outpost_to_hub);
  }

  @Override
  public Pose2d getStartingPose() {
    if (right_to_outpost != null) {
      return right_to_outpost.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
