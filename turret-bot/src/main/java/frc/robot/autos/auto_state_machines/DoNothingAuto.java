package frc.robot.autos.auto_state_machines;

import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.robot_manager.RobotManager;

public class DoNothingAuto extends BaseImperativeAuto<DoNothingAutoState> {
  public DoNothingAuto(RobotManager robot, Trailblazer trailblazer) {
    super(DoNothingAutoState.DO_NOTHING, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Pose2d.kZero;
  }
}
