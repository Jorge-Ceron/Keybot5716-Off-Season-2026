// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.RobotCore;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private RobotCore buildRobotCore() {
    if (RobotBase.isSimulation()) {
      return new RobotContainerSim();
    } else {
      return new RobotContainer();
    }
  }

  private Command autonomousCommand;

  private final RobotCore robotContainer;

  public static Optional<Alliance> alliance = Optional.of(Alliance.Blue);

  static final int REAL_TIME_PRIORITY = 2;
  static final int NON_REAL_TIME_PRIORITY = 1;
  boolean isEnabled = false;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        if (!DriverStation.isFMSAttached()) {
          Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.addDataReceiver(new WPILOGWriter());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
    if (!Logger.hasReplaySource()) {
      RobotController.setTimeSource(RobotController::getFPGATime);
    }

    robotContainer = buildRobotCore();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SignalLogger.enableAutoLogging(false);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    if (DriverStation.isEnabled()) {
      Threads.setCurrentThreadPriority(true, REAL_TIME_PRIORITY);
    } else {
      Threads.setCurrentThreadPriority(false, NON_REAL_TIME_PRIORITY);
    }
    CommandScheduler.getInstance().run();
    // robotContainer.getRobotVisualizer().updateRobotVisualizer();
    robotContainer.getRobotState().updateLogger();
    robotContainer.getRobotVisualizer().updateRobotVisualizer();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance() : alliance;
  }

  @Override
  public void disabledExit() {
    // robotContainer.getVisionSubsystem().setUseVision(true);
    SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Threads.setCurrentThreadPriority(true, REAL_TIME_PRIORITY);
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Threads.setCurrentThreadPriority(true, REAL_TIME_PRIORITY);
    SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());

    if (RobotBase.isSimulation() && !isEnabled) {
      SimulatedArena.getInstance().placeGamePiecesOnField();
      // SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(null));
    }

    if (!isEnabled) {
      isEnabled = true;
    }

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(autonomousCommand);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    robotContainer.getDriveSubsystem().resetOdometry(FieldConstants.getSimulatedStartPos());
    // robotContainer.getDriveSubsystem().resetOdometry(new Pose2d(0, 0, new Rotation2d()));

    // AIRobotSimulated.startOpponentRobotSimulations();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    if (robotContainer instanceof RobotContainerSim) {
      RobotContainerSim simContainer = (RobotContainerSim) robotContainer;

      if (simContainer.getSimRobotState() != null) {
        simContainer.getSimRobotState().updateSim();
      }
    }
  }
}
