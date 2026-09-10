package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterHoodIOSim implements ShooterHoodIO {
  // Configuración física del Hood
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), // Motor que mueve el hood
          100.0, // Reducción (Gearing) - Ajusta según tu robot
          0.001, // Momento de inercia (Jkgm^2)
          0.3, // Longitud del brazo (metros)
          Math.toRadians(0), // Ángulo mínimo (Radianes)
          Math.toRadians(70), // Ángulo máximo (Radianes)
          true, // ¿Simular gravedad?
          Math.toRadians(0) // Ángulo inicial
          );

  private double appliedVolts = 0.0;
  private double desiredPositionRad = 0.0;
  private boolean isClosedLoop = false;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Si estamos en modo posición, simulamos un PID simple
    if (isClosedLoop) {
      double error = desiredPositionRad - hoodSim.getAngleRads();
      appliedVolts = MathUtil.clamp(error * 40.0, -12.0, 12.0); // Kp de 40 como ejemplo
    }

    // Actualizar la simulación (ciclo de 20ms)
    hoodSim.setInput(appliedVolts);
    hoodSim.update(0.020);

    // Enviar datos a los inputs para que el Subsystem los lea
    inputs.position = Math.toDegrees(hoodSim.getAngleRads());
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setPosition(double positionDeg) {
    this.desiredPositionRad = Math.toRadians(positionDeg);
    this.isClosedLoop = true;
  }

  @Override
  public void stop() {
    this.appliedVolts = 0.0;
    this.isClosedLoop = false;
  }

  @Override
  public void runOpenLoop(double output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runOpenLoop'");
  }

  @Override
  public void setVoltage(double volts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public void setNeutralModeBreak(boolean enable) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setNeutralModeBreak'");
  }

  @Override
  public void resetEncoder() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetEncoder'");
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPID'");
  }
}
