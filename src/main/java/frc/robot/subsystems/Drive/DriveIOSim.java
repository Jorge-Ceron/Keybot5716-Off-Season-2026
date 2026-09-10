package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.simulation.SimulatedRobotState;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class DriveIOSim extends DriveIOCTRE {
  private SimulatedRobotState simRobotState = null;
  private static final double SIM_LOOP_PERIOD = 0.005;
  private Notifier simNotifier = null;
  private double lastSimTime;

  MapleSimSwerveDrivetrain simSwerve = null;

  Pose2d lastConsumedPose = null;
  Consumer<SwerveDriveState> simTelemetryConsumer =
      state -> {
        if (simRobotState == null) return;
        if (Constants.useMapleSim && simSwerve != null) {
          state.Pose = simSwerve.mapleSimDrive.getSimulatedDriveTrainPose();
        }
        simRobotState.addFieldToRobot(state.Pose);
        telemetryConsumer.accept(state);
      };

  public DriveIOSim(
      RobotState robotState,
      SimulatedRobotState simRobotState,
      SwerveDrivetrainConstants swerveConstants,
      @SuppressWarnings("rawtypes") SwerveModuleConstants... modules) {
    super(robotState, swerveConstants, modules);
    this.simRobotState = simRobotState;

    registerTelemetry(simTelemetryConsumer);
    startThread();
  }

  public void resetOdometry(Pose2d pose) {
    if (Constants.useMapleSim && simSwerve != null) {
      simSwerve.mapleSimDrive.setSimulationWorldPose(pose);
      Timer.delay(0.05);
    }
    super.resetOdometry(pose);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    super.updateInputs(inputs);

    if (Constants.useMapleSim && simSwerve != null) {
      Pose2d p = simSwerve.mapleSimDrive.getSimulatedDriveTrainPose();
      inputs.Pose = p;
    }

    var pose = simRobotState.getLatestFieldToRobot();
    if (pose != null) {
      Logger.recordOutput("Drive/Sim/SimPose", simRobotState.getLatestFieldToRobot());
    }
  }

  @SuppressWarnings("unchecked")
  public void startThread() {
    if (Constants.useMapleSim) {
      simSwerve =
          new MapleSimSwerveDrivetrain(
              Seconds.of(SIM_LOOP_PERIOD),
              Kilograms.of(DriveConstants.ROBOT_MASS),
              Inches.of(DriveConstants.BUMPER_LENGTH_INCHES),
              Inches.of(DriveConstants.BUMPER_WIDTH_INCHES),
              DCMotor.getKrakenX60(1),
              DCMotor.getKrakenX60(1),
              1,
              getModuleLocations(),
              getPigeon2(),
              getModules(),
              SimTunerConstants.FrontLeft,
              SimTunerConstants.FrontRight,
              SimTunerConstants.BackLeft,
              SimTunerConstants.BackRight);
      simNotifier = new Notifier(simSwerve::update);
    } else {
      lastSimTime = Utils.getCurrentTimeSeconds();
      simNotifier =
          new Notifier(
              () -> {
                final double time = Utils.getCurrentTimeSeconds();
                double deltaTime = time - lastSimTime;
                lastSimTime = time;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
              });
    }
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  public MapleSimSwerveDrivetrain getSimSwerve() {
    return simSwerve;
  }
}
