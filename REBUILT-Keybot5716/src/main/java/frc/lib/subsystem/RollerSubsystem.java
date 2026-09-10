package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

  private DesiredState desiredState = DesiredState.STOPPED;
  private RollersState rollersState = RollersState.STOPPING;

  private double desiredVoltage;

  // RobotState robotState;

  public enum DesiredState {
    STOPPED,
    FORWARD,
    REVERSE
  }

  private enum RollersState {
    STOPPING,
    FORWARDING,
    REVERSING
  }

  private RollersState setStateTransition() {
    return switch (desiredState) {
      case STOPPED -> RollersState.STOPPING;
      case FORWARD -> RollersState.FORWARDING;
      case REVERSE -> RollersState.REVERSING;
    };
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public void setDesiredStateWithVoltage(DesiredState desiredState, double desiredVoltage) {
    this.desiredState = desiredState;
    this.desiredVoltage = desiredVoltage;
  }
}
