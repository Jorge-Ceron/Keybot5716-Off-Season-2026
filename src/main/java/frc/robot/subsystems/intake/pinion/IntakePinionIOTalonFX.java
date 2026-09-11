package frc.robot.subsystems.intake.pinion;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

public class IntakePinionIOTalonFX implements IntakePinionIO {
  private final TalonFX PinMotor;
  private final VoltageOut voltageOut = new VoltageOut(Volts.zero());
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Angle> positionIntake;
  private final StatusSignal<AngularVelocity> velocityIntake;
  private final StatusSignal<AngularAcceleration> accelerationIntake;
  private final StatusSignal<Current> supplyCurrentIntake;
  private final StatusSignal<Current> statorCurrentIntake;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakePinionIOTalonFX() {
    PinMotor = new TalonFX(IDs.INTAKE_PINION_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.DutyCycleNeutralDeadband = 0.04;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = 1.2;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kG = 0.8;

    config.Audio.BeepOnBoot = true;

    PinMotor.getConfigurator().apply(config);

    appliedVolts = PinMotor.getMotorVoltage();
    positionIntake = PinMotor.getRotorPosition();
    velocityIntake = PinMotor.getRotorVelocity();
    accelerationIntake = PinMotor.getAcceleration();
    supplyCurrentIntake = PinMotor.getSupplyCurrent();
    statorCurrentIntake = PinMotor.getStatorCurrent();
    tempCelsius = PinMotor.getDeviceTemp();

    TalonFXSignalFrequencies.updateFrequencyTalonFX(
        appliedVolts,
        positionIntake,
        velocityIntake,
        accelerationIntake,
        supplyCurrentIntake,
        statorCurrentIntake,
        tempCelsius);

    PinMotor.optimizeBusUtilization();
    PinMotor.setPosition(0.0);
  }

  public void setPosition(double position) {
    PinMotor.setControl(positionVoltage.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    PinMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void stopMotor() {
    PinMotor.stopMotor();
  }

  @Override
  public void updateInputs(IntakePinionIOInputs inputs) {
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
