package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class DriveTelemetry {
  private final double maxSpeed;
  private final String prefix = "Drive/Telemetry/";

  private final Field2d field = new Field2d();

  public DriveTelemetry(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    SmartDashboard.putData(field);
  }

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d lastPose = Pose2d.kZero;
  private double lastTime = Logger.getTimestamp();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] moduleSpeeds =
      new MechanismLigament2d[] {
        moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] moduleDirections =
      new MechanismLigament2d[] {
        moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      };

  public void telemeterize(SwerveDriveState state) {
    if (state == null || state.Pose == null || state.ModuleStates == null) return;

    Pose2d pose = state.Pose;
    Logger.recordOutput(prefix + "Pose", pose);

    Pose3d pose3d =
        new Pose3d(
            pose.getX(),
            pose.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
    Logger.recordOutput(prefix + "Pose3d", pose3d);

    if (DriverStation.isDisabled() || Robot.isSimulation()) {
      field.setRobotPose(pose);
    }

    /* Telemeterize the robot's general speeds */
    double currentTime = Logger.getTimestamp();
    double delta = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
    lastPose = pose;
    Translation2d velocities = distanceDiff.div(delta);

    Logger.recordOutput(prefix + "Speed", velocities.getNorm());
    Logger.recordOutput(prefix + "VelocityX", velocities.getX());
    Logger.recordOutput(prefix + "VelocityY", velocities.getY());
    Logger.recordOutput(prefix + "OdomPeriod", state.OdometryPeriod);
    Logger.recordOutput("OdomPeriod", state.OdometryPeriod);
    Logger.recordOutput("ModuleStatesCurrent", state.ModuleStates);

    /* Telemeterize the module's states */
    for (int i = 0; i < 4; ++i) {
      moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));
    }
  }
}
