package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class Auto4Test extends AutoBuilder {

  private final PathPlannerPath test8;
  private final PathPlannerPath test9;
  private final PathPlannerPath test10;

  public Auto4Test() {

    test8 = loadPath("Testing8");
    test9 = loadPath("Testing9");
    test10 = loadPath("Testing10");

    addCommands(Commands.deadline(new PathPlannerAuto("TestAuto")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
    return getPathPosesList(test8, test9, test10);
  }

  @Override
<<<<<<< Updated upstream
  public Pose2d getStartingPose() {
    if (test8 != null) {
      return test8.getStartingDifferentialPose();
=======
  public List<Posed2d> getStartingPose() {

    if (test8 != null) {

      return getStartingDifferentialPose();
>>>>>>> Stashed changes
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
