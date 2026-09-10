package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.subsystems.shooter.rollers.ShooterRollersIOSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakePivotIOSim implements IntakePivotIO {

  private final IntakeSimulation intakeSimulation;
  private ShooterRollersIOSim shooterRollersIOSim = new ShooterRollersIOSim();

  /**
   * Este método sirve para configurar la especificaciones del intake que se va a mostrar simulado
   *
   * @param driveTrain Sirve para tener en cuenta en qué base estara este intake
   */
  public IntakePivotIOSim(AbstractDriveTrainSimulation driveTrain) {
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Este es el tipo de objeto que el intake va a recoger
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            // The width of the intake (CHANGEEEEE)
            Meters.of(0.6985), // 20 inches ancho : 25 22.5
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.26),
            // The intake is mounted on the front? side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to ? balls?
            30);
  }

  /**
   * Con esto podemos hacer que extienda o que retraiga el intake
   *
   * @param runIntake sirve si es que queremos que se extienda o se retraiga
   */
  public void setRunning(boolean runIntake) {
    if (runIntake)
      intakeSimulation
          .startIntake(); // Extends the intake out from the chassis frame and starts detecting
    // contacts with game pieces
    else
      intakeSimulation
          .stopIntake(); // Retracts the intake into the chassis frame, disabling game piece
    // collection
  }

  /**
   * @return si hay algun fuel en el intake
   */
  public boolean isFuelInsideIntake() {
    return intakeSimulation.getGamePiecesAmount()
        != 0; // True if there is a game piece in the intake
  }

  public int getFuelAmount() {
    return intakeSimulation.getGamePiecesAmount();
  }

  public IntakeSimulation getIntakeSimulation() {
    return intakeSimulation;
  }

  public boolean isRunning() {
    return intakeSimulation.isRunning();
  }

  public void resetIntake() {
    intakeSimulation.setGamePiecesCount(0);
  }
  /*
  public void launchFuel() {
    if (isFuelInsideIntake()) {
      shooterRollersIOSim.shoot();
    }
    intakeSimulation.setGamePiecesCount(getFuelAmount() - 1);
  }
    */

  /**
   * public void launchFuel() { // if there is a fuel in the intake, it will be removed and return
   * true; otherwise, returns // false if (intakeSimulation.obtainGamePieceFromIntake()) {} //
   * ShooterIOSim.launchFuel(); //notify the simulated flywheels to launch a fuel (We need the //
   * shooter to be aware of this) }
   */
  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void stopMotor() {}

  @Override
  public void setPosition(double position) {}
}
