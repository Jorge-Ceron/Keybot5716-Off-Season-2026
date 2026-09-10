package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class AutoMech extends AutoBuilder {
  private final PathPlannerPath test;

  public AutoMech() {
    test = loadPath("JUST_MECH");

    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("MECH_AUTO"))));
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
