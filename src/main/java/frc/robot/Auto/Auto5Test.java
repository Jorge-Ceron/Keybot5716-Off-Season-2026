package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class Auto5Test extends AutoBuilder {

  private final PathPlannerPath test11;
  private final PathPlannerPath test12;
  private final PathPlannerPath test13;

  public Auto5Test() {

    test11 = loadPath("Testing11");
    test12 = loadPath("Testing12");
    test13 = loadPath("Testing13");

    addCommands(Commands.deadline(new PathPlannerAuto("TestAuto")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
    return getPathPosesList(test11, test12, test13);
  }

  @Override
<<<<<<< Updated upstream
  public Pose2d getStartingPose() {
    if (test11 != null) {
      return test11.getStartingDifferentialPose();
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
