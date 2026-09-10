package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;

public class CommandSwerveDrivetrain {
  SwerveDrivetrainConstants drivetrainConstants;
  SwerveModuleConstants<?, ?, ?>[] moduleConstants;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    this.drivetrainConstants = drivetrainConstants;

    if (Constants.useMapleSim) {
      this.moduleConstants = MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules);
    } else {
      this.moduleConstants = modules;
    }
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
