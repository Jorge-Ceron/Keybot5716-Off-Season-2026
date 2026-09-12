package frc.robot.subsystems.intake.pivot;

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

public class IntakePivotSubsystem extends SubsystemBase {
  private IntakePivotIO pivotIO;

  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();

  private TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2.0, 2.0);

  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(profileConstraints);

  private ProfiledPIDController controller =
      new ProfiledPIDController(10, 0, 0.08, profileConstraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  private static final LoggedTunableNumber pivotVolts =
      new LoggedTunableNumber("Intake/Pinion/PinionVolts", 2.0);

  private IntakeState intakeState = IntakeState.STOPPING_PIVOT;
  private DesiredState desiredState = DesiredState.STOPPPED_PIVOT;

  private RobotState robotState;

  public enum DesiredState {
    FORWARD_PIVOT,
    REVERSE_PIVOT,
    STOPPPED_PIVOT,
    pIN,
    pOUT
  }

  private enum IntakeState {
    FORWARDING_PIVOT,
    REVERSING_PIVOT,
    STOPPING_PIVOT,
    pINING,
    pOUTING
  }

  public IntakePivotSubsystem(IntakePivotIO pivotIO, RobotState robotState) {
    this.pivotIO = pivotIO;
    this.robotState = robotState;
    controller.setTolerance(0.02);
    DataProcessor.initDataProcessor(
        () -> {
          synchronized (pivotInputs) {
            pivotIO.updateInputs(pivotInputs);
          }
        },
        pivotIO);
  }

  @Override
  public void periodic() {
    synchronized (pivotInputs) {
      Logger.processInputs("Intake/Pinion/PinionInputs", pivotInputs);

      Logger.recordOutput("Intake/Pinion/DesiredState", desiredState);
      Logger.recordOutput("Intake/Pinion/CurrentState", intakeState);

      double rot = pivotInputs.position;
      double angleRad = rotationsToIntakeRadians(rot);

      robotState.setArmAngle(angleRad);

      intakeState = setStateTransition();
      applyStates();
    }
  }

  private IntakeState setStateTransition() {
    return switch (desiredState) {
      case FORWARD_PIVOT -> IntakeState.FORWARDING_PIVOT;
      case REVERSE_PIVOT -> IntakeState.REVERSING_PIVOT;
      case STOPPPED_PIVOT -> IntakeState.STOPPING_PIVOT;
      case pIN -> IntakeState.pINING;
      case pOUT -> IntakeState.pOUTING;
    };
  }

  private void applyStates() {
    switch (intakeState) {
      case pINING:
        setPosition(SuperstructureConstants.IntakeConstants.pIN);
        break;

      case pOUTING:
        setPosition(SuperstructureConstants.IntakeConstants.pOUT);
        break;

      case FORWARDING_PIVOT:
        runPinion(pivotVolts.get());
        break;

      case REVERSING_PIVOT:
        runPinion(-pivotVolts.get());
        break;

      case STOPPING_PIVOT:
        stopPinion();
        break;
    }
  }

  public boolean isOut() {
    return MathUtil.isNear(IntakeConstants.OUT, pivotInputs.position, 1.0);
  }

  public boolean isIn() {
    return MathUtil.isNear(IntakeConstants.IN, pivotInputs.position, 0.08);
  }

  public void setPosition(double position) {
    pivotIO.setPosition(position);
  }

  public void runPinion(double voltage) {
    pivotIO.setVoltage(voltage);
  }

  public void stopPinion() {
    pivotIO.stopMotor();
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
