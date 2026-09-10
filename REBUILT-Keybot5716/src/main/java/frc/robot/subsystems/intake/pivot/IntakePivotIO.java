package frc.robot.subsystems.intake.pivot;

import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO extends DataProcessor.IODataRefresher {
  @AutoLog
  public class IntakePivotIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double position = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double tempCelcius = 0.0;
  }

  void updateInputs(IntakePivotIOInputs inputs);

  void setVoltage(double voltage);

  void stopMotor();

  void setPosition(double position);

  @Override
  default void refreshData() {}
}
