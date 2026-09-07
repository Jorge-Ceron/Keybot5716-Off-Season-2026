package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.TalonFXSignalFrequencies;
import frc.robot.subsystems.superstructure.SuperstructureConstants.IDs;

public class IntakeRollerIOTalonFX implements IntakeRollersIO {
  private final TalonFX motor;
  private final VoltageOut voltageOut = new VoltageOut(Volts.zero());
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<AngularVelocity> velocityRollers;
  private final StatusSignal<AngularAcceleration> accelerationRollers;
  private final StatusSignal<Current> supplyCurrentRollers;
  private final StatusSignal<Current> statorCurrentRollers;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakeRollerIOTalonFX() {
    motor = new TalonFX(IDs.INTAKE_ROLLER_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.DutyCycleNeutralDeadband = 0.04;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    config.Slot0.kP = 0.3;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.013;

    config.Audio.BeepOnBoot = true;

    motor.getConfigurator().apply(config);

    appliedVolts = motor.getMotorVoltage();
    velocityRollers = motor.getRotorVelocity();
    accelerationRollers = motor.getAcceleration();
    supplyCurrentRollers = motor.getSupplyCurrent();
    statorCurrentRollers = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();

    TalonFXSignalFrequencies.updateFrequencyTalonFX(
        appliedVolts,
        velocityRollers,
        accelerationRollers,
        supplyCurrentRollers,
        statorCurrentRollers,
        tempCelsius);

    motor.optimizeBusUtilization();
    motor.setPosition(0.0);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setVelocity(double rps) {
    motor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            appliedVolts,
            velocityRollers,
            accelerationRollers,
            supplyCurrentRollers,
            statorCurrentRollers,
            tempCelsius);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.velocity = velocityRollers.getValueAsDouble();
    inputs.acceleration = accelerationRollers.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentRollers.getValueAsDouble();
    inputs.statorCurrent = statorCurrentRollers.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void refreshData() {
    BaseStatusSignal.refreshAll(
        appliedVolts,
        velocityRollers,
        accelerationRollers,
        supplyCurrentRollers,
        statorCurrentRollers,
        tempCelsius);
  }
}
