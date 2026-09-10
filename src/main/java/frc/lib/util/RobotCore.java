package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;

public interface RobotCore {
  RobotState getRobotState();

  Visualizer getRobotVisualizer();

  Command getAutonomousCommand();

  DriveSubsystem getDriveSubsystem();
}
