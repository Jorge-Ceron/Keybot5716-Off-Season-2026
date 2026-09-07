package frc.robot.subsystems.intake.rollers;

import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO extends DataProcessor.IODataRefresher {

  @AutoLog
  public class IntakeRollersIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double tempCelcius = 0.0;
  }

  void updateInputs(IntakeRollersIOInputs inputs);

  void setVoltage(double voltage);

  void setVelocity(double rps);

  void stopMotor();

  @Override
  default void refreshData() {}
}
