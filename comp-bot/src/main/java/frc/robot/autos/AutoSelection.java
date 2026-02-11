package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.IntegrationTest;
import frc.robot.autos.auto_state_machines.TestAuto;
import frc.robot.autos.auto_state_machines.TrenchRFeed2Auto;
import frc.robot.autos.auto_state_machines.TrenchRShootClimb2Auto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  TRENCH_FEED_2(TrenchRFeed2Auto::new),
  TRENCH_SHOOT_CLIMB(TrenchRShootClimb2Auto::new),
  INTEGRATION_TEST(IntegrationTest::new),
  TEST(TestAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
