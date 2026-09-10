package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class Auto3Test extends AutoBuilder {

  private final PathPlannerPath test5;
  private final PathPlannerPath test6;
  private final PathPlannerPath test7;

  public Auto3Test() {

    test5 = loadPath("Testing5");
    test6 = loadPath("Testing6");
    test7 = loadPath("Testing7");

    addCommands(Commands.deadline(new PathPlannerAuto("TestAuto")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
    return getPathPosesList(test5, test6, test7);
  }

  @Override
<<<<<<< Updated upstream
  public Pose2d getStartingPose() {
    if (test5 != null) {
      return test5.getStartingDifferentialPose();
=======
  public List<Posed2d> getStartingPose() {

    if (test5 != null) {

      return getStartingDifferentialPose();
>>>>>>> Stashed changes
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
