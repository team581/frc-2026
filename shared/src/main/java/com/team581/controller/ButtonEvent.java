package com.team581.controller;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class ButtonEvent extends BooleanEvent {
  private final BooleanEvent onPress;
  private final BooleanEvent onRelease;

  public ButtonEvent(EventLoop loop, BooleanEvent button) {
    super(loop, button);
    this.onPress = button.rising();
    this.onRelease = button.falling();
  }

  public ButtonEvent onPress(Runnable action) {
    onPress.ifHigh(action);
    return this;
  }

  public ButtonEvent onRelease(Runnable action) {
    onRelease.ifHigh(action);
    return this;
  }
}
