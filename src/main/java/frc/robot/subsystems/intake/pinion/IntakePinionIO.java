package frc.robot.subsystems.intake.pinion;

import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePinionIO extends DataProcessor.IODataRefresher {
  @AutoLog
  public class IntakePinionIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double position = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double tempCelcius = 0.0;
  }

  void updateInputs(IntakePinionIOInputs inputs);

  void setVoltage(double voltage);

  void stopMotor();

  void setPosition(double position);

  @Override
  default void refreshData() {}
}
