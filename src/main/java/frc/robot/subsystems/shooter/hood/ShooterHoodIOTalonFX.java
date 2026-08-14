package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.TalonFXSignalFrequencies;
import frc.robot.subsystems.superstructure.SuperstructureConstants.IDs;

public class ShooterHoodIOTalonFX implements ShooterHoodIO {

  private final TalonFX motor;
  private TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageOutRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Angle> positionIntake;
  private final StatusSignal<AngularVelocity> velocityIntake;
  private final StatusSignal<AngularAcceleration> accelerationIntake;
  private final StatusSignal<Current> supplyCurrentIntake;
  private final StatusSignal<Current> statorCurrentIntake;
  private final StatusSignal<Temperature> tempCelsius;

  public ShooterHoodIOTalonFX() {
    motor = new TalonFX(IDs.SHOOTER_HOOD_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.DutyCycleNeutralDeadband = 0.04;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.95;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.1;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    config.Slot0.kP = 7.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.Audio.BeepOnBoot = true;

    motor.getConfigurator().apply(config);

    appliedVolts = motor.getMotorVoltage();
    positionIntake = motor.getRotorPosition();
    velocityIntake = motor.getRotorVelocity();
    accelerationIntake = motor.getAcceleration();
    supplyCurrentIntake = motor.getSupplyCurrent();
    statorCurrentIntake = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();

    TalonFXSignalFrequencies.updateFrequencyTalonFX(
        appliedVolts,
        positionIntake,
        velocityIntake,
        accelerationIntake,
        supplyCurrentIntake,
        statorCurrentIntake,
        tempCelsius);

    motor.optimizeBusUtilization();
    motor.setPosition(0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            appliedVolts,
            positionIntake,
            velocityIntake,
            accelerationIntake,
            supplyCurrentIntake,
            statorCurrentIntake,
            tempCelsius);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.position = positionIntake.getValueAsDouble();
    inputs.velocity = velocityIntake.getValueAsDouble();
    inputs.acceleration = accelerationIntake.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentIntake.getValueAsDouble();
    inputs.statorCurrent = statorCurrentIntake.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runOpenLoop(double output) {
    motor.setControl(new DutyCycleOut(output));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageOutRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void setNeutralModeBreak(boolean enable) {
    config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void resetEncoder() {
    motor.setPosition(0.0);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void refreshData() {
    BaseStatusSignal.refreshAll(
        appliedVolts,
        positionIntake,
        velocityIntake,
        accelerationIntake,
        supplyCurrentIntake,
        statorCurrentIntake,
        tempCelsius);
  }
}
