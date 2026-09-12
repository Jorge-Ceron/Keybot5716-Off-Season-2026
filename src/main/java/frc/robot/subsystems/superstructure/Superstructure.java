package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.pinion.IntakePinionSubsystem;
import frc.robot.subsystems.intake.pivot.*;
import frc.robot.subsystems.intake.rollers.IntakeRollersSubsystem;
import frc.robot.subsystems.shooter.ShootCalculator;
import frc.robot.subsystems.shooter.hood.ShooterHoodSubsystem;
import frc.robot.subsystems.shooter.rollers.ShooterRollersSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants;
import frc.robot.subsystems.transfer.TransferSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final DriveSubsystem driveSub;
  private final IntakePinionSubsystem intakePinionSub;
  private final IntakePivotSubsystem intakePivotSub;
  private final IntakeRollersSubsystem intakeRollersSub;
  private final TransferSubsystem transferSub;
  private final ShooterHoodSubsystem shooterHoodSub;
  private final ShooterRollersSubsystem shooterRollerSub;
  private final ShootCalculator shootCalculator;
  private final RobotState robotState;

  private Timer scoreTimer = new Timer();
  private boolean timerStarted = false;

  private SuperstructureStates currentState = SuperstructureStates.DEFAULT;
  private SuperstructureStates desiredState = SuperstructureStates.DEFAULT;

  private ShootCalculator.LaunchPreset activePreset = null;

  public Superstructure(
      DriveSubsystem driveSub,
      IntakePivotSubsystem intakePivotSub,
      IntakePinionSubsystem intakePinionSub,
      IntakeRollersSubsystem intakeRollersSub,
      TransferSubsystem transferSub,
      ShooterHoodSubsystem shooterHoodSub,
      ShooterRollersSubsystem shooterRollersSub,
      ShootCalculator shootCalculator,
      RobotState robotState) {
    this.driveSub = driveSub;
    this.intakePivotSub = intakePivotSub;
    this.intakePinionSub = intakePinionSub; // CORREGIDO: Asignación faltante
    this.intakeRollersSub = intakeRollersSub;
    this.transferSub = transferSub;
    this.shooterHoodSub = shooterHoodSub;
    this.shooterRollerSub = shooterRollersSub;
    this.shootCalculator = shootCalculator;
    this.robotState = robotState;
  }

  public void setDesiredState(SuperstructureStates targetState) {
    this.desiredState = targetState;
  }

  @Override
  public void periodic() {
    if (currentState != desiredState) {
      currentState = desiredState;
    }
    Logger.recordOutput("Superstructure/DesiredState", desiredState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);

    tryStates();
  }

  private void tryStates() {
    switch (currentState) {
      case DEFAULT:
        def();
        break;

      case HOME:
        home();
        break;

      case INTAKE:
        intake();
        break;

      case SCORE:
        score();
        break;

      case TAXI:
        taxi();
        break;

      case EJECT:
        def();
        break;

      case SHOOTER_TEST:
        presetShoot();
        break;

      case MANUAL_SCORE:
        manualScore();
        break;

      case MANUAL_TAXI:
        manualTaxi();
        break;
    }
  }

  private void def() {
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    shooterHoodSub.setDesiredState(ShooterHoodSubsystem.DesiredState.HOME);
    shooterRollerSub.setDesiredState(ShooterRollersSubsystem.DesiredState.STOPPED);
  }

  private void home() {
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
    // CORREGIDO: Usar intakePivotSub y su propio Enum (DesiredState.HOME o STOPPPED_PIVOT)
    intakePivotSub.setDesiredState(IntakePivotSubsystem.DesiredState.STOPPPED_PIVOT);
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    shooterHoodSub.setDesiredState(ShooterHoodSubsystem.DesiredState.HOME);
    shooterRollerSub.setDesiredState(ShooterRollersSubsystem.DesiredState.STOPPED);
  }

  private void intake() {
    driveSub.setState(DriveSubsystem.DesiredState.MANUAL_FIELD_DRIVE);
    // CORREGIDO: Usar intakePivotSub y su propio Enum (ej. pIN / OUT)
    intakePivotSub.setDesiredState(IntakePivotSubsystem.DesiredState.pIN);
    transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    shooterHoodSub.setDesiredState(ShooterHoodSubsystem.DesiredState.STOPPED);
    shooterRollerSub.setDesiredState(ShooterRollersSubsystem.DesiredState.STOPPED);

    // CORREGIDO: Método de verificación adaptado al Pivot
    if (intakePivotSub.isOut()) {
      intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.FORWARD_ROLLERS);
    } else {
      intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    }
  }

  private void score() {
    driveSub.setDesiredPointToLock(FieldConstants.getHubShootingPose().getTranslation());
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    shooterRollerSub.setDesiredState(ShooterRollersSubsystem.DesiredState.FORWARD_ROLLERS);
    shooterHoodSub.setDesiredState(ShooterHoodSubsystem.DesiredState.CALC_POS_TO_SCORE);

    if (driveSub.isAlignedToPoint() && shooterRollerSub.atDesiredVelocity()) {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.OSCILLATE_FORWARD);
    }
  }

  private void manualScore() {
    shooterHoodSub.setAngle(0.15);
    shooterRollerSub.setCustom(ShooterConstants.SCORE_RPS);

    if (shooterRollerSub.atDesiredVelocity()) {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.OSCILLATE_FORWARD);
    }
  }

  private void manualTaxi() {
    shooterHoodSub.setAngle(ShooterConstants.OUT_TEST);
    shooterRollerSub.setCustom(55.0);

    if (shooterRollerSub.atDesiredVelocity()) {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.OSCILLATE_FORWARD);
    }
  }

  private void taxi() {
    driveSub.setDesiredRotationToLock(
        new Rotation2d(robotState.isRedAlliance() ? (Math.PI / 2) : (Math.PI + (Math.PI / 2))));
    intakeRollersSub.setDesiredState(IntakeRollersSubsystem.DesiredState.STOPPED);
    shooterRollerSub.setCustom(ShooterConstants.TAXI_RPS);
    shooterHoodSub.setDesiredState(ShooterHoodSubsystem.DesiredState.CALC_POS_TO_TAXI);

    if (driveSub.isAlignedToAngle()
        && shooterHoodSub.isOut()
        && shooterRollerSub.atDesiredVelocity()) {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.OSCILLATE_FORWARD);
    } else {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.STOPPED);
    }
  }

  public void presetShoot() {
    driveSub.setDesiredPointToLock(FieldConstants.getHubShootingPose().getTranslation());

    shooterHoodSub.setAngle(0.15);
    shooterRollerSub.setCustom(ShooterConstants.SCORE_RPS);

    if (shooterRollerSub.atDesiredVelocity()) {
      transferSub.setDesiredState(TransferSubsystem.DesiredState.OSCILLATE_FORWARD);
    }
  }

  public Command setPresetCommand(ShootCalculator.LaunchPreset preset) {
    return Commands.runOnce(
        () -> {
          this.activePreset = preset;
          setDesiredState(SuperstructureStates.SHOOTER_TEST);
        },
        this);
  }

  public Command setCommand(SuperstructureStates superState) {
    return new InstantCommand(() -> setDesiredState(superState));
  }

  public Command setCommand(SuperstructureStates taxiState, SuperstructureStates hubState) {
    return Commands.either(
        setCommand(taxiState), setCommand(hubState), () -> robotState.passedTrench());
  }

  public Command setCommand(SuperstructureStates taxiState, ShootCalculator.LaunchPreset preset) {
    return Commands.either(
        setCommand(taxiState), setPresetCommand(preset), () -> robotState.passedTrench());
  }
}
