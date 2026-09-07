package frc.robot.subsystems.transfer;

import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface TransferIO extends DataProcessor.IODataRefresher {

  @AutoLog
  public class TransferIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double tempCelcius = 0.0;
  }

  void updateInputs(TransferIOInputs inputs);

  void setVelocity(double rps);

  void stopMotor();

  @Override
  default void refreshData() {}
}
