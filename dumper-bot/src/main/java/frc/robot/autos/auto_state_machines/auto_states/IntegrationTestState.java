package frc.robot.autos.auto_state_machines.auto_states;

import com.google.common.collect.ImmutableMap;

public enum IntegrationTestState {
  SEGMENT_1_DRIVE_TO_START,
  SEGMENT_2_CLOSE_CENTERED_WITH_HUB,
  SEGMENT_3_BACK_CENTERED_WITH_HUB,
  SEGMENT_4_RIGHT_TRENCH,
  SEGMENT_5_LEFT_RAMP;

  private static final ImmutableMap<IntegrationTestState, IntegrationTestState> NEXT_STATE =
      ImmutableMap.of(
          SEGMENT_1_DRIVE_TO_START,
          SEGMENT_2_CLOSE_CENTERED_WITH_HUB,
          SEGMENT_2_CLOSE_CENTERED_WITH_HUB,
          SEGMENT_3_BACK_CENTERED_WITH_HUB,
          SEGMENT_3_BACK_CENTERED_WITH_HUB,
          SEGMENT_4_RIGHT_TRENCH,
          SEGMENT_4_RIGHT_TRENCH,
          SEGMENT_5_LEFT_RAMP);

  private static final ImmutableMap<IntegrationTestState, IntegrationTestState> PREVIOUS_STATE =
      ImmutableMap.of(
          SEGMENT_2_CLOSE_CENTERED_WITH_HUB,
          SEGMENT_1_DRIVE_TO_START,
          SEGMENT_3_BACK_CENTERED_WITH_HUB,
          SEGMENT_2_CLOSE_CENTERED_WITH_HUB,
          SEGMENT_4_RIGHT_TRENCH,
          SEGMENT_3_BACK_CENTERED_WITH_HUB,
          SEGMENT_5_LEFT_RAMP,
          SEGMENT_4_RIGHT_TRENCH);

  public IntegrationTestState nextState() {
    return NEXT_STATE.getOrDefault(this, this);
  }

  public IntegrationTestState previousState() {
    return PREVIOUS_STATE.getOrDefault(this, this);
  }
}
