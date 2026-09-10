package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveConstants {

  public static final double ROBOT_MASS = 40;
  public static final double BUMPER_LENGTH_INCHES = 35;
  public static final double BUMPER_WIDTH_INCHES = 35;

  public static final double DRIVE_TO_POSE_STATIC_FRICTION_CONSTANT = 0.02;

  public static final PIDController teleopAutoAllignController = new PIDController(3.0, 0, 0.1);
  public static final PIDController autoAutoAllignController = new PIDController(3.0, 0, 0.1);

  public static final double maxVelocityToAutoAllign = Units.feetToMeters(8.0);

  public static final CommandSwerveDrivetrain SWERVE_DRIVETRAIN =
      Robot.isSimulation()
          ? SimTunerConstants.createDrivetrain()
          : TunerConstants.createDrivetrain();

  public static final double MAX_SPEED = 4.58;
  public static final double WHEEL_COF = 1.0;

  public static ModuleConfig moduleConfig =
      new ModuleConfig(
          SWERVE_DRIVETRAIN.getModuleConstants()[0].WheelRadius,
          MAX_SPEED,
          WHEEL_COF,
          DCMotor.getKrakenX60(1),
          SWERVE_DRIVETRAIN.getModuleConstants()[0].DriveMotorGearRatio,
          SWERVE_DRIVETRAIN.getModuleConstants()[0].SlipCurrent,
          1);

  public static RobotConfig robotConfig =
      new RobotConfig(
          Constants.ROBOT_MASS_KG,
          Constants.ROBOT_MOI,
          moduleConfig,
          new Translation2d(0.27686, 0.27686),
          new Translation2d(0.27686, -0.27686),
          new Translation2d(-0.27686, 0.27686),
          new Translation2d(-0.27686, -0.27686));

  public static PPHolonomicDriveController pathPlannerController =
      new PPHolonomicDriveController(
          new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));
}
