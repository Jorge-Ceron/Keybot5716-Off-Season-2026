package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.TalonFXSignalFrequencies;
import frc.robot.subsystems.superstructure.SuperstructureConstants.IDs;

public class TrasnferIOTalonFX implements TransferIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<AngularVelocity> velocityRollers;
  private final StatusSignal<AngularAcceleration> accelerationRollers;
  private final StatusSignal<Current> supplyCurrentRollers;
  private final StatusSignal<Current> statorCurrentRollers;
  private final StatusSignal<Temperature> tempCelsius;

  public TrasnferIOTalonFX() {
    leaderMotor = new TalonFX(IDs.TRANSFER_ID);
    followerMotor = new TalonFX(IDs.TRANSFER_FOLLOWER_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
    config.Slot0.kV = 0.15;

    config.Audio.BeepOnBoot = true;

    leaderMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    appliedVolts = leaderMotor.getMotorVoltage();
    velocityRollers = leaderMotor.getRotorVelocity();
    accelerationRollers = leaderMotor.getAcceleration();
    supplyCurrentRollers = leaderMotor.getSupplyCurrent();
    statorCurrentRollers = leaderMotor.getStatorCurrent();
    tempCelsius = leaderMotor.getDeviceTemp();

    TalonFXSignalFrequencies.updateFrequencyTalonFX(
        appliedVolts,
        velocityRollers,
        accelerationRollers,
        supplyCurrentRollers,
        statorCurrentRollers,
        tempCelsius);

    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();

    leaderMotor.setPosition(0.0);
    followerMotor.setPosition(0.0);
  }

  public void setPosition(double position) {
    leaderMotor.setPosition(position);
    followerMotor.setPosition(position);
  }

  @Override
  public void setVelocity(double rps) {
    leaderMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stopMotor() {
    leaderMotor.stopMotor();
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
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
