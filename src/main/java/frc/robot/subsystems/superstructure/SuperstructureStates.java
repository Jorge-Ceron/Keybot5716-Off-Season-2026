package frc.robot.subsystems.superstructure;

import java.util.EnumSet;
import java.util.Set;

public enum SuperstructureStates {
  DEFAULT,
  HOME,
  INTAKE,
  SCORE,
  MANUAL_SCORE,
  MANUAL_TAXI,
  TAXI,
  SHOOTER_TEST,
  EJECT;

  public boolean isScoring() {
    return this == SCORE || this == SHOOTER_TEST;
  }

  public boolean isIntaking() {
    return this == INTAKE;
  }

  public Set<SuperstructureStates> allowedNextStates() {
    switch (this) {
      case DEFAULT:
        return EnumSet.of(INTAKE, SCORE);

      case INTAKE:
        return EnumSet.of(HOME, EJECT);

      case HOME:
        return EnumSet.of(INTAKE, SCORE, EJECT);

      case SCORE:
        return EnumSet.of(HOME, DEFAULT);

      case MANUAL_SCORE:
        return EnumSet.of(HOME, DEFAULT);

      case MANUAL_TAXI:
        return EnumSet.of(HOME, DEFAULT);

      case SHOOTER_TEST:
        return EnumSet.of(HOME, DEFAULT);

      case TAXI:
        return EnumSet.of(HOME, DEFAULT);

      default:
        return EnumSet.allOf(SuperstructureStates.class);
    }
  }
}
