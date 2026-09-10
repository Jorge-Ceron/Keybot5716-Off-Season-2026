package frc.robot.subsystems.shooter.hood;

import frc.lib.util.DataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO extends DataProcessor.IODataRefresher {

  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double position = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double tempCelcius = 0.0;
  }

  void updateInputs(HoodIOInputs inputs);

  void stop();

  void runOpenLoop(double output);

  void setVoltage(double volts);

  void setPosition(double position);

  void setNeutralModeBreak(boolean enable);

  void resetEncoder();

  void setPID(double kP, double kI, double kD);

  @Override
  default void refreshData() {}
}
