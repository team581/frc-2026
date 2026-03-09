package frc.robot.hub_activity;

import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.config.DSOptions;
import frc.robot.util.scheduling.SubsystemPriority;

public class HubActivity extends StateMachineSubsystem<HubActivityState> {
  private static final double FORCE_SCORE_TRANSITION_TIMEOUT = 3.0;
  private final DoubleSubscriber tunableHubStateOffset =
      DogLog.tunable("HubActivity/MatchTimeOffset", 2.0);

  private final Timer teleopTimer = new Timer();
  private double timeSinceMatchStart = 0.0;

  private double timeUntilNextShift = 0.0;
  private boolean actualHubActive = true;
  private double scoringShooterTOF = 0.0;
  private boolean tofBasedHubActive = true;

  private boolean forceScoreTransitionEndOfActiveHub = false;

  public HubActivity() {
    super(SubsystemPriority.HUB_ACTIVITY, HubActivityState.DEFAULT_STATE);

    teleopTimer.start();
  }

  public boolean ableToForceScoreTransitionEndOfActiveHub() {
    return forceScoreTransitionEndOfActiveHub;
  }

  public boolean getActualHubActive() {
    return actualHubActive;
  }

  public Color getHubStateColor() {
    if (getTOFBasedHubActive()) {
      if (getActualHubActive()) {
        return Color.kLimeGreen;
      } else {
        boolean isGreen = Math.floor(teleopTimer.get() / 0.1) % 2 == 0;
        return isGreen ? Color.kLimeGreen : Color.kBlack;
      }
    } else {
      return Color.kRed;
    }
  }

  public boolean getTOFBasedHubActive() {
    return tofBasedHubActive;
  }

  @Override
  public void teleopInit() {
    teleopTimer.reset();
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
    timeSinceMatchStart = teleopTimer.get() + FmsUtil.MATCH_TIME_AT_TELEOP_START;
    timeUntilNextShift = FmsUtil.timeUntilNextShift(timeSinceMatchStart);

    SmartDashboard.putString("HubActivity/CurrentShift", FmsUtil.currentShift(timeSinceMatchStart));
    SmartDashboard.putString("HubActivity/Active", getHubStateColor().toHexString());
    SmartDashboard.putNumber("HubActivity/TimeUntilNextShift", timeUntilNextShift);

    actualHubActive = calculateActualHubActive();
    tofBasedHubActive = calculateTOFBasedHubActive();
    forceScoreTransitionEndOfActiveHub =
        DSOptions.USE_HUB_STATE.get()
            && actualHubActive
            && timeUntilNextShift < FORCE_SCORE_TRANSITION_TIMEOUT;
  }

  @Override
  protected void whileInState(HubActivityState state) {
    DogLog.log("HubActivity/TimeSinceMatchStart", timeSinceMatchStart);
    DogLog.log("HubActivity/TimeSinceTeleopEnable", teleopTimer.get());
    DogLog.log("HubActivity/TimeUntilNextShift", timeUntilNextShift);
    DogLog.log(
        "HubActivity/Scoring/ScoreTransition/ForceScoreTransitionEndOfActiveHub",
        ableToForceScoreTransitionEndOfActiveHub());
    DogLog.log("HubActivity/ActualHubActive", actualHubActive);
    DogLog.log("HubActivity/TOFBasedHubActive", tofBasedHubActive);
  }
}
