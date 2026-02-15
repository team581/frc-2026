package frc.robot.lights;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Optional;

public enum LightsState {
  // Blink slow duration -> 0.25, blink fast duration -> 0.08
  PLACEHOLDER(Color.kBlack, 0.0),
  BLINK(Color.kWhite, 0.08),

  ERROR(Color.kRed, 0.08),
  UNHOMED(Color.kOrangeRed, 0.25),
  HOMED_NO_TAGS(Color.kYellow),
  HOMED_SEES_TAGS(Color.kGreen),

  LIMELIGHTS_DEAD(Color.kRed, 0.08),

  WAITING_TO_SHOOT(Color.kYellow, 0.25),
  SHOOT(Color.kGreen, 0.08),
  SHOT_DONE(Color.kWhite, 0.08),

  IDLE_INTAKE_NOT_FULL(Color.kYellow, 0.25),
  IDLE_INTAKE_FULL(Color.kWhite, 0.08),

  CLIMB_1(Color.kRed),
  CLIMB_2(Color.kOrange),
  CLIMB_3(Color.kYellow),
  CLIMB_4(Color.kGreen),
  CLIMB_5(Color.kBlue),
  CLIMB_6(Color.kIndigo),
  CLIMB_7(Color.kViolet);

  public final Optional<StrobeAnimation> stateBlinkRequest;
  public final Optional<SolidColor> stateColorRequest;

  public final Color color;
  public final double duration;

  LightsState(Color color) {
    this.color = color;
    this.duration = Double.POSITIVE_INFINITY;
    this.stateBlinkRequest = Optional.empty();
    this.stateColorRequest = Optional.of(new SolidColor(0, 399).withColor(new RGBWColor(color)));
  }

  LightsState(Color color, double duration) {
    this.color = color;
    this.duration = duration;
    this.stateBlinkRequest =
        Optional.of(
            new StrobeAnimation(0, 399)
                .withColor(new RGBWColor(color))
                .withFrameRate(1 / duration));
    this.stateColorRequest = Optional.empty();
  }

  public boolean blinks() {
    return stateBlinkRequest.isPresent();
  }
}
