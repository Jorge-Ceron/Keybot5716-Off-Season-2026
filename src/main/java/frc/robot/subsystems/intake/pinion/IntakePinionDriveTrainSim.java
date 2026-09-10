package frc.robot.subsystems.intake.pinion;

import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class IntakePinionDriveTrainSim extends AbstractDriveTrainSimulation {

  public IntakePinionDriveTrainSim(DriveTrainSimulationConfig config, Pose2d initialPose) {
    super(config, initialPose);
  }

  @Override
  public void simulationSubTick() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'simulationSubTick'");
  }
}
