package frc.robot.hub_activity;

import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.config.DSOptions;
import frc.robot.util.scheduling.SubsystemPriority;

public class HubActivity extends StateMachineSubsystem<HubActivityState> {
  private static final String LIME_GREEN_HEX = Color.kLimeGreen.toHexString();
  private static final String BLACK_HEX = Color.kBlack.toHexString();
  private static final String RED_HEX = Color.kRed.toHexString();

  private final DoubleSubscriber tunableHubStateOffset =
      DogLog.tunable("HubActivity/FieldHubDelay", 2.0);

  private final Timer teleopTimer = new Timer();
  private final Timer autoTimer = new Timer();
  private double timeSinceMatchStart = 0.0;

  private double timeUntilNextShift = 0.0;
  private boolean actualHubActive = true;
  private double scoringShooterTOF = 0.0;
  private boolean tofBasedHubActive = true;

  private boolean shouldBeastMode = false;

  public HubActivity() {
    super(SubsystemPriority.HUB_ACTIVITY, HubActivityState.DEFAULT_STATE);

    autoTimer.start();
    teleopTimer.start();
  }

  @Override
  public void autonomousInit() {
    autoTimer.restart();
    timeSinceMatchStart = FmsUtil.MATCH_TIME_AT_AUTO_START;
  }

  public boolean getActualHubActive() {
    return actualHubActive;
  }

  public String getHubStateColorHex() {
    if (getTOFBasedHubActive()) {
      if (getActualHubActive()) {
        return LIME_GREEN_HEX;
      } else {
        boolean isGreen = Math.floor(teleopTimer.get() / 0.1) % 2 == 0;
        return isGreen ? LIME_GREEN_HEX : BLACK_HEX;
      }
    } else {
      return RED_HEX;
    }
  }

  public boolean getTOFBasedHubActive() {
    return DSOptions.PIT_FUNCTIONALITY.getAsBoolean() || tofBasedHubActive;
  }

  public boolean shouldBeastMode() {
    return shouldBeastMode;
  }

  @Override
  public void teleopInit() {
    teleopTimer.restart();
    timeSinceMatchStart = FmsUtil.MATCH_TIME_AT_TELEOP_START;
  }

  public void updateShooterScoringTOF(double scoringShooterTOF) {
    this.scoringShooterTOF = scoringShooterTOF;
  }

  private boolean calculateActualHubActive() {
    // Hub always active in auto or if DS option is turned off
    if (!DSOptions.USE_HUB_STATE.get() || DriverStation.isAutonomous()) {
      return true;
    }

    return FmsUtil.isHubActive(timeSinceMatchStart, DSOptions.DEFAULT_WON_AUTO.getAsBoolean());
  }

  private boolean calculateTOFBasedHubActive() {
    // Hub always active in auto or if DS option is turned off
    if (!DSOptions.USE_HUB_STATE.get() || DriverStation.isAutonomous()) {
      return true;
    }

    var risingTOFOffsetHubActive =
        FmsUtil.isHubActive(
            timeSinceMatchStart + scoringShooterTOF + tunableHubStateOffset.get(),
            DSOptions.DEFAULT_WON_AUTO.getAsBoolean());

    var fallingOffsetHubActive =
        FmsUtil.isHubActive(
            timeSinceMatchStart + scoringShooterTOF - tunableHubStateOffset.get(),
            DSOptions.DEFAULT_WON_AUTO.getAsBoolean());

    return risingTOFOffsetHubActive || fallingOffsetHubActive;
  }

  @Override
  protected void collectInputs() {
    if (DriverStation.isAutonomous()) {
      timeSinceMatchStart = autoTimer.get();
    } else {
      timeSinceMatchStart = teleopTimer.get() + FmsUtil.MATCH_TIME_AT_TELEOP_START;
    }
    timeSinceMatchStart = teleopTimer.get() + FmsUtil.MATCH_TIME_AT_TELEOP_START;
    timeUntilNextShift =
        FmsUtil.timeUntilNextShift(timeSinceMatchStart, DSOptions.DEFAULT_WON_AUTO.getAsBoolean());

    actualHubActive = calculateActualHubActive();
    tofBasedHubActive = calculateTOFBasedHubActive();

    shouldBeastMode =
        DSOptions.USE_HUB_STATE.getAsBoolean()
            && (timeSinceMatchStart > (FmsUtil.ENDGAME_DURATION - 5.0));
  }

  @Override
  protected void whileInState(HubActivityState state) {
    if (DriverStation.isDisabled()) {
      teleopTimer.stop();
      teleopTimer.reset();
      autoTimer.stop();
      autoTimer.reset();
    }

    // Driver station logging
    DogLog.forceNt.log("HubActivity/CurrentShift", FmsUtil.currentShift(timeSinceMatchStart));
    DogLog.forceNt.log("HubActivity/Active", getHubStateColorHex());
    DogLog.forceNt.log("HubActivity/TimeUntilNextShift", timeUntilNextShift);

    DogLog.log("HubActivity/TimeSinceMatchStart", timeSinceMatchStart);
    DogLog.log("HubActivity/TimeSinceTeleopEnable", teleopTimer.get());
    DogLog.log("HubActivity/ActualHubActive", actualHubActive);
    DogLog.log("HubActivity/TOFBasedHubActive", tofBasedHubActive);
    DogLog.log("HubActivity/ShouldBeastMode", shouldBeastMode);
  }
}
