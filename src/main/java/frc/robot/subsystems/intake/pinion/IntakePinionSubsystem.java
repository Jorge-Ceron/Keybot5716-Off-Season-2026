package frc.robot.subsystems.intake.pinion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.util.DataProcessor;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakePinionSubsystem extends SubsystemBase {
  private IntakePinionIO pinionIO;

  // Renamed the variable name to 'pinionInputs' from 'pinionInputsAutoLogged' for more readability.
  private final IntakePinionIOInputsAutoLogged pinionInputs = new IntakePinionIOInputsAutoLogged();

  private TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2.0, 2.0);

  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(profileConstraints);

  private ProfiledPIDController controller =
      new ProfiledPIDController(10, 0, 0.08, profileConstraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  private static final LoggedTunableNumber pinionVolts =
      new LoggedTunableNumber("Intake/Pinion/PinionVolts", 2.0);

  private IntakeState intakeState = IntakeState.STOPPING_PINION;
  private DesiredState desiredState = DesiredState.STOPPPED_PINION;

  private RobotState robotState;

  public enum DesiredState {
    FORWARD_PINION,
    REVERSE_PINION,
    STOPPPED_PINION,
    IN,
    OUT
  }

  private enum IntakeState {
    FORWARDING_PINION,
    REVERSING_PINION,
    STOPPING_PINION,
    INING,
    OUTING
  }

  public IntakePinionSubsystem(IntakePinionIO pinionIO, RobotState robotState) {
    this.pinionIO = pinionIO;
    this.robotState = robotState;
    controller.setTolerance(0.02);
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (pinionInputs) {
            pinionIO.updateInputs(pinionInputs);
          }
        },
        pinionIO);
  }

  @Override
  public void periodic() {
    synchronized (pinionInputs) {
      Logger.processInputs("Intake/Pinion/PinionInputs", pinionInputs);

      Logger.recordOutput("Intake/Pinion/DesiredState", desiredState);
      Logger.recordOutput("Intake/Pinion/CurrentState", intakeState);

      double rot = pinionInputs.position;
      double angleRad = rotationsToIntakeRadians(rot);

      robotState.setArmAngle(angleRad);

      intakeState = setStateTransition();
      applyStates();
    }
  }

  private IntakeState setStateTransition() {
    return switch (desiredState) {
      case FORWARD_PINION -> IntakeState.FORWARDING_PINION;
      case REVERSE_PINION -> IntakeState.REVERSING_PINION;
      case STOPPPED_PINION -> IntakeState.STOPPING_PINION;
      case IN -> IntakeState.INING;
      case OUT -> IntakeState.OUTING;
    };
  }

  private void applyStates() {
    switch (intakeState) {
      case INING:
        setPosition(SuperstructureConstants.IntakeConstants.IN);
        break;

      case OUTING:
        setPosition(SuperstructureConstants.IntakeConstants.OUT);
        break;

      case FORWARDING_PINION:
        runPinion(pinionVolts.get());
        break;

      case REVERSING_PINION:
        runPinion(-pinionVolts.get());
        break;

      case STOPPING_PINION:
        stopPinion();
        break;
    }
  }

  public boolean isOut() {
    return MathUtil.isNear(IntakeConstants.OUT, pinionInputs.position, 1.0);
  }

  public boolean isIn() {
    return MathUtil.isNear(IntakeConstants.IN, pinionInputs.position, 0.08);
  }

  public void setPosition(double position) {
    pinionIO.setPosition(position);
  }

  public void runPinion(double voltage) {
    pinionIO.setVoltage(voltage);
  }

  public void stopPinion() {
    pinionIO.stopMotor();
  }

  public void setDesiredState(DesiredState desiredState) {
    this.desiredState = desiredState;
  }

  public double rotationsToIntakeRadians(double rot) {
    double minRot = 0.0;
    double maxRot = Math.abs(IntakeConstants.OUT);

    double scale = (Math.PI / 2.0) / (maxRot - minRot); // 90° = π/2
    double rad = (rot - minRot) * scale;

    return MathUtil.clamp(rad, 0.0, Math.PI / 2.0); // hola soy george67676767676767
  }
}
