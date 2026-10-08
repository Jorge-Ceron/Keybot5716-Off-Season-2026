package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class CommandSwerveDrivetrain {
  SwerveDrivetrainConstants drivetrainConstants;
  SwerveModuleConstants<?, ?, ?>[] moduleConstants;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    this.drivetrainConstants = drivetrainConstants;
  }

  /**
   * @return The constraints for the Swerve Drivetrain
   */
  public SwerveDrivetrainConstants getDrivetrainConstants() {
    return drivetrainConstants;
  }

  /**
   * @return The constants for each module
   */
  public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
    return moduleConstants;
  }
}
