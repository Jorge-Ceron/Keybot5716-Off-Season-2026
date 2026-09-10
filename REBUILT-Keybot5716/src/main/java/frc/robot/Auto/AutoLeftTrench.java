package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class AutoLeftTrench extends AutoBuilder {
  private final PathPlannerPath left_to_center;
  private final PathPlannerPath left_center_intake;
  private final PathPlannerPath left_center_to_hub;
  private final PathPlannerPath left_hub_to_center;
  private final PathPlannerPath left_center_intake2;
  private final PathPlannerPath last;

  public AutoLeftTrench() {
    left_to_center = loadPath("LEFT_TRENCH_TO_CENTER");
    left_center_intake = loadPath("LEFTCENTER_INTAKE");
    left_center_to_hub = loadPath("LEFT_CENTER_TO_HUB");
    left_hub_to_center = loadPath("LEFT_CENTER_INTAKE2");
    left_center_intake2 = loadPath("LEFT_INTAKE22");
    last = loadPath("LEFTINTAKE_TO_HUB2");

    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("LEFT_TRENCH_AUTO"))));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(
        left_to_center,
        left_center_intake,
        left_center_to_hub,
        left_hub_to_center,
        left_center_intake2,
        last);
  }

  @Override
  public Pose2d getStartingPose() {
    if (left_to_center != null) {
      return left_to_center.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
