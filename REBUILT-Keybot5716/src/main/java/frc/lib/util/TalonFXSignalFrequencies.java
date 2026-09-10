package frc.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public final class TalonFXSignalFrequencies {
  private TalonFXSignalFrequencies() {}

  public static void updateFrequencyTalonFX(
      StatusSignal<Voltage> appliedVolts,
      StatusSignal<Angle> position,
      StatusSignal<AngularVelocity> velocity,
      StatusSignal<AngularAcceleration> acceleration,
      StatusSignal<Current> supplyCurrent,
      StatusSignal<Current> statorCurrent,
      StatusSignal<Temperature> temperature) {
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity);
    appliedVolts.setUpdateFrequency(50.0);
    BaseStatusSignal.setUpdateFrequencyForAll(25.0, supplyCurrent, statorCurrent);
    temperature.setUpdateFrequency(5.0);
    acceleration.setUpdateFrequency(50.0);
  }

  public static void updateFrequencyTalonFX(
      StatusSignal<Voltage> appliedVolts,
      StatusSignal<AngularVelocity> velocity,
      StatusSignal<AngularAcceleration> acceleration,
      StatusSignal<Current> supplyCurrent,
      StatusSignal<Current> statorCurrent,
      StatusSignal<Temperature> temperature) {
    velocity.setUpdateFrequency(100.0);
    appliedVolts.setUpdateFrequency(50.0);
    BaseStatusSignal.setUpdateFrequencyForAll(25.0, supplyCurrent, statorCurrent);
    temperature.setUpdateFrequency(5.0);
    acceleration.setUpdateFrequency(50.0);
  }
}
