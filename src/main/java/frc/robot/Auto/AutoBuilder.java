package frc.robot.Auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public abstract class AutoBuilder extends SequentialCommandGroup {
  public abstract Pose2d getStartingPose();

  public abstract List<Pose2d> getPathPoses();

  public List<Pose2d> getPathPosesList(PathPlannerPath... paths) {
    return Stream.of(paths)
        .filter(Objects::nonNull)
        .flatMap(path -> path.getPathPoses().stream())
        .collect(Collectors.toList());
  }

  public PathPlannerPath loadPath(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception err) {
      DriverStation.reportError(
          "Failed to load path: " + pathName + " - " + err.getMessage(), err.getStackTrace());
      return null;
    }
  }
}
