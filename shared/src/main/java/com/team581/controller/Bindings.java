package com.team581.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import java.util.function.Function;

public class Bindings {
  private final XboxController controller;
  private final BooleanEvent enabledEvent;
  private final EventLoop eventLoop;

  public Bindings(EventLoop eventLoop, BooleanEvent enabledEvent, XboxController controller) {
    this.controller = controller;
    this.enabledEvent = enabledEvent;
    this.eventLoop = eventLoop;
  }

  public ButtonBinding a() {
    return button(controller::a);
  }

  public ButtonBinding b() {
    return button(controller::b);
  }

  public ButtonBinding back() {
    return button(controller::back);
  }

  public ButtonBinding button(Function<EventLoop, BooleanEvent> rawEvent) {
    return new ButtonBinding(eventLoop, rawEvent.apply(eventLoop).and(enabledEvent));
  }

  public ButtonBinding leftBumper() {
    return button(controller::leftBumper);
  }

  public ButtonBinding leftTrigger() {
    return button(controller::leftTrigger);
  }

  public ButtonBinding rightBumper() {
    return button(controller::rightBumper);
  }

  public ButtonBinding rightTrigger() {
    return button(controller::rightTrigger);
  }

  public ButtonBinding start() {
    return button(controller::start);
  }

  public ButtonBinding x() {
    return button(controller::x);
  }

  public ButtonBinding y() {
    return button(controller::y);
  }
}
