package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class IntakePivotDriveTrainSim extends AbstractDriveTrainSimulation {

  public IntakePivotDriveTrainSim(DriveTrainSimulationConfig config, Pose2d initialPose) {
    super(config, initialPose);
  }

  @Override
  public void simulationSubTick() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'simulationSubTick'");
  }
}
