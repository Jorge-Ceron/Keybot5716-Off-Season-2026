package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class AutoComp1 extends AutoBuilder {

  private final PathPlannerPath AutoComp1;
  private final PathPlannerPath AutoComp2;
  private final PathPlannerPath AutoComp3;
  private final PathPlannerPath AutoComp4;
  private final PathPlannerPath AutoComp5;
  private final PathPlannerPath AutoComp6;

  public AutoComp1() {

    AutoComp1 = loadPath("AutoComp1.1");
    AutoComp2 = loadPath("AutoComp1.2");
    AutoComp3 = loadPath("AutoComp1.3");
    AutoComp4 = loadPath("AutoComp1.4");
    AutoComp5 = loadPath("AutoComp1.5");
    AutoComp6 = loadPath("AutoComp1.6");

    addCommands(Commands.deadline(new PathPlannerAuto("AutoComp")));
  }

  @Override
  public List<Pose2d> getPathPoses() {
    return getPathPosesList(AutoComp1, AutoComp2, AutoComp3, AutoComp4, AutoComp5, AutoComp6);
  }

  @Override
  public Pose2d getStartingPose() {
    if (AutoComp1 != null) {
      return AutoComp1.getStartingDifferentialPose();
    }

    DriverStation.reportError("Path is null", true);
    return new Pose2d();
  }
}
