package frc.robot.auto.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.rollers.ShooterRollersSubsystem;

public class ShootAuto extends Command {

  private ShooterRollersSubsystem rollers;

  public ShootAuto(ShooterRollersSubsystem rollers) {
    this.rollers = rollers;

    addRequirements(rollers);
  }

  @Override
  public void initialize() {
    rollers.setVelocityRollers(50.5);
  }

  @Override
  public void execute() {
    rollers.setVelocityRollers(50.5);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
