package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.util.DataProcessor;
import frc.robot.subsystems.superstructure.SuperstructureConstants.TransferConstants;
import org.littletonrobotics.junction.Logger;

public class TransferSubsystem extends SubsystemBase {
  private final TransferIO transferIO;

  private final TransferIOInputsAutoLogged transferInputs = new TransferIOInputsAutoLogged();

  private DesiredState desiredState = DesiredState.STOPPED;
  private TransferState transferState = TransferState.STOPPING;

  private final Timer oscillationTimer = new Timer();
  boolean oscillating = false;

  private static final LoggedTunableNumber rollerRPS =
      new LoggedTunableNumber("Transfer/Rollers/RollerRPS", 55.0);

  public enum DesiredState {
    STOPPED,
    FORWARD,
    REVERSE,
    OSCILLATE_FORWARD
  }

  private enum TransferState {
    STOPPING,
    FORWARDING,
    REVERSING
  }

  public TransferSubsystem(TransferIO transferIO) {
    this.transferIO = transferIO;
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (transferInputs) {
            transferIO.updateInputs(transferInputs);
          }
        },
        transferIO);
  }

  @Override
  public void periodic() {
    synchronized (transferInputs) {
      Logger.processInputs("Transfer/TransferInputs", transferInputs);

      Logger.recordOutput("Transfer/DesiredState", desiredState);
      Logger.recordOutput("Transfer/CurrentState", transferState);
      transferState = setStateTransition();
      applyStates();
    }
  }

  private TransferState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> TransferState.STOPPING;
      case FORWARD -> TransferState.FORWARDING;
      case REVERSE -> TransferState.REVERSING;
      case OSCILLATE_FORWARD -> TransferState.FORWARDING;
    };
  }

  private void applyStates() {
    switch (transferState) {
      case FORWARDING:
        if (desiredState == DesiredState.OSCILLATE_FORWARD) {

          double cycleTime = oscillationTimer.get() % 1.2;

          if (cycleTime < 0.2) {
            setVelocity((-TransferConstants.FORWARD_RPS) * 0.8);
          } else {
            setVelocity(TransferConstants.FORWARD_RPS);
          }

        } else {
          setVelocity(TransferConstants.FORWARD_RPS);
        }
        break;

      case REVERSING:
        setVelocity(-TransferConstants.FORWARD_RPS);
        break;

      case STOPPING:
        stop();
        break;
    }
  }

  private void stop() {
    transferIO.stopMotor();
  }

  private void setVelocity(double rps) {
    transferIO.setVelocity(rps);
  }

  public void setDesiredState(DesiredState desiredState) {
    if (this.desiredState != desiredState) {

      if (desiredState == DesiredState.OSCILLATE_FORWARD) {
        oscillationTimer.stop();
        oscillationTimer.reset();
        oscillationTimer.start();
      }
    }

    this.desiredState = desiredState;
  }

  public void startOscillatingForward() {
    oscillationTimer.stop();
    oscillationTimer.reset();
    oscillationTimer.start();
    oscillating = true;
    desiredState = DesiredState.FORWARD;
  }

  public void stopOscillating() {
    oscillating = false;
    oscillationTimer.stop();
  }
}
