package frc.robot.auto;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static java.util.Collections.emptyList;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public class NoneAuto extends AutoBuilder {

  public NoneAuto() {
    addCommands(none());
  }

  public Pose2d getStartingPose() {
    return new Pose2d();
  }

  public List<Pose2d> getPathPoses() {
    return emptyList();
  }
}
