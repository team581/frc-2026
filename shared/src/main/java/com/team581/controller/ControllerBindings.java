package com.team581.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import java.util.function.Function;

public class ControllerBindings {
  private final XboxController controller;
  private final BooleanEvent enabledEvent;
  private final EventLoop eventLoop;

  public ControllerBindings(
      EventLoop eventLoop, BooleanEvent enabledEvent, XboxController controller) {
    this.controller = controller;
    this.enabledEvent = enabledEvent;
    this.eventLoop = eventLoop;
  }

  public ButtonEvent a() {
    return button(controller::a);
  }

  public ButtonEvent b() {
    return button(controller::b);
  }

  public ButtonEvent back() {
    return button(controller::back);
  }

  public ButtonEvent button(Function<EventLoop, BooleanEvent> rawEvent) {
    return new ButtonEvent(eventLoop, rawEvent.apply(eventLoop).and(enabledEvent));
  }

  public ButtonEvent leftBumper() {
    return button(controller::leftBumper);
  }

  public ButtonEvent leftTrigger() {
    return button(controller::leftTrigger);
  }

  public ButtonEvent rightBumper() {
    return button(controller::rightBumper);
  }

  public ButtonEvent rightTrigger() {
    return button(controller::rightTrigger);
  }

  public ButtonEvent start() {
    return button(controller::start);
  }

  public ButtonEvent x() {
    return button(controller::x);
  }

  public ButtonEvent y() {
    return button(controller::y);
  }
}
