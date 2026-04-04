package frc.robot.autos.auto_state_machines.auto_state;

public enum IntegratedAutoState {
  INTAKE_ACROSS_MIDLINE,
  // Fallback segment for non-cluster map (robot fully on our side)
  DEFAULT_SECOND_INTAKE_SEGMENT,

  // Lane 1 for cluster map (robot mostly on our side)
  INTAKE_LANE_1,

  // Lane 2 for cluster map (robot mostly on opponent side)
  INTAKE_LANE_2,

  // Trench lane for cluster map (trench and then alliance lane)
  INTAKE_TRENCH_LANE,
  DRIVE_BACK_1,
  DRIVE_BACK_2,
  DRIVE_BACK_TO_NEUTRAL_ZONE,
  SHOOT_1,
  SHOOT_2,
  DONE,

  // For when we get beached on a ball
  STUCK_ON_BALL_RECOVERY
}
