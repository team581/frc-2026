package com.team581.controller;

public enum JoystickState {
  NOT_SEEN_INPUT_TELEOP(false),
  SEEN_INPUT_TELEOP(true);

  public final boolean value;

  private JoystickState(boolean value) {
    this.value = value;
  }
}
