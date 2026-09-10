package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SuperstructureConstants {

  public static class IDs {
    // -- INTAKE
    public static final int INTAKE_PIVOT_ID = 16;
    public static final int INTAKE_ROLLER_ID = 17;

    // -- SHOOTER
    public static final int SHOOTER_HOOD_ID = 31;
    public static final int SHOOTER_ROLLERS_ID = 30;

    // -- TRANSFER
    public static final int TRANSFER_ID = 22;
  }

  public static class IntakeConstants {
    // -- PIVOT CONSTANTS
    public static final double IN = -0.0;
    public static final double OUT = -5.2;

    // -- ROLLER CONSTANTS
    public static final double ZERO_RVOLTAGE = 0.0;
    public static final double FORWARD_RVOLTAGE = 8.0;
    public static final double REVERSE_RVOLTAGE = 8.0;

    public static final double NONE_RPS = 0.0;
    public static final double FORWARD_RPS = 25.0;
    public static final double REVERSE_RPS = 20.0;
  }

  public static class ShooterConstants {
    public static final double TAXI_RPS = 55.0;
    public static final double SCORE_RPS = 48.0;

    public static final double HOME = 0.2;
    public static final double IN_TEST = 0.2;
    public static final double OUT_TEST = 0.87;
    public static final double MID_TEST = 0.3;

    public static Transform2d robotToLauncher =
        new Transform2d(new Translation2d(-0.26, 0.13), new Rotation2d());
  }

  public static class TransferConstants {
    public static final double FORWARD_RPS = 28.0;
  }
}
