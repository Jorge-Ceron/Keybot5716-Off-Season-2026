package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DataProcessor;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterHoodSubsystem extends SubsystemBase {
  private final ShooterHoodIO hoodIO;

  private HoodState hoodState = HoodState.STOPPING;
  private DesiredState desiredState = DesiredState.STOPPED;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private double desiredAngle = 0.0;

  public enum DesiredState {
    HOME,
    CALC_POS_TO_SCORE,
    CALC_POS_TO_TAXI,
    STOPPED
  }

  private enum HoodState {
    HOMING,
    POSITIONED_TO_SCORE,
    POSITIONED_TO_TAXI,
    STOPPING
  }

  public ShooterHoodSubsystem(ShooterHoodIO hoodIO) {
    this.hoodIO = hoodIO;
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (hoodInputs) {
            hoodIO.updateInputs(hoodInputs);
          }
        },
        hoodIO);
  }

  @Override
  public void periodic() {
    synchronized (hoodInputs) {
      Logger.processInputs("Shooter/Hood/HoodInputs", hoodInputs);

      Logger.recordOutput("Shooter/Hood/DesiredState", desiredState);
      Logger.recordOutput("Shooter/Hood/CurrentState", hoodState);

      hoodState = setStateTransition();
      applyStates();
    }
  }

  private HoodState setStateTransition() {
    return switch (desiredState) {
      case HOME -> HoodState.HOMING;
      case CALC_POS_TO_SCORE -> HoodState.POSITIONED_TO_SCORE;
      case CALC_POS_TO_TAXI -> HoodState.POSITIONED_TO_TAXI;
      case STOPPED -> HoodState.STOPPING;
    };
  }

  private void applyStates() {
    switch (hoodState) {
      case HOMING:
        setPosition(ShooterConstants.HOME);
        break;
      case POSITIONED_TO_SCORE:
        setPosition(desiredAngle);
        break;

      case POSITIONED_TO_TAXI:
        setPosition(ShooterConstants.OUT_TEST);
        break;

      case STOPPING:
        stop();
        break;
    }
  }

  public boolean isOut() {
    return MathUtil.isNear(ShooterConstants.OUT_TEST, hoodInputs.position, 0.2);
  }

  public boolean atHome() {
    return MathUtil.isNear(ShooterConstants.HOME, hoodInputs.position, 0.08);
  }

  public void stop() {
    hoodIO.stop();
  }

  public void setPosition(double pos) {
    hoodIO.setPosition(pos);
  }

  public void setAngle(double angle) {
    this.desiredAngle = angle;
    setDesiredState(DesiredState.CALC_POS_TO_SCORE);
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public double getAngle() {
    return hoodInputs.position;
  }
}
