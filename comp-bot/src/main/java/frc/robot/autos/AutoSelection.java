package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.IntegrationTest;
import frc.robot.autos.auto_state_machines.RightPullSwoopShootAuto;
// import frc.robot.autos.auto_state_machines.RightStraightFeedAuto;
import frc.robot.autos.auto_state_machines.RightStraightShootClimbAuto;
//import frc.robot.autos.auto_state_machines.RightSwoopShootClimbAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  // RIGHT_STRAIGHT_FEED(RightStraightFeedAuto::new),
  RIGHT_STRAIGHT_SHOOT_CLIMB(RightStraightShootClimbAuto::new),

  //RIGHT_SWOOP_SHOOT_CLIMB(RightSwoopShootClimbAuto::new),
  RIGHT_PULL_SWOOP_SHOOT_CLIMB(RightPullSwoopShootAuto::new),
  INTEGRATION_TEST(IntegrationTest::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
