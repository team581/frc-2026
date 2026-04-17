package frc.robot.imu;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.team581.mechanisms.imu.BaseImuSubsystem;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.util.scheduling.SubsystemPriority;

public class Imu extends BaseImuSubsystem {
  private static final DoubleSubscriber COLLISION_G_FORCE_THRESHOLD =
      DogLog.tunable("Imu/CollisionGForceThreshold", 0.8);

  public final BumpCrossingTracker bumpCrossingTracker;
  private final LinearFilter pigeonXAccelFilter = LinearFilter.movingAverage(10);
  private final LinearFilter pigeonYAccelFilter = LinearFilter.movingAverage(10);

  private double pigeonXPrevAccel = 0.0;
  private double pigeonYPrevAccel = 0.0;
  private double pigeonGForce = 0.0;
  private double maxGForceDetected = Double.NEGATIVE_INFINITY;
  private double lastUpdateTime = 0.0;

  public Imu(SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(SubsystemPriority.IMU, drivetrain);

    // TODO(@jonahsnider): This should use localization#getPose() but I think that makes a circular
    // import which I don't want to deal with right now
    this.bumpCrossingTracker =
        new BumpCrossingTracker(
            () -> getPitch(),
            () -> getRoll(),
            () -> driveState.Pose,
            translation ->
                drivetrain.resetPose(new Pose2d(translation, driveState.Pose.getRotation())));
  }

  @Override
  public void collectInputs() {
    super.collectInputs();

    double pigeonFilteredXAccel =
        pigeonXAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    double pigeonFilteredYAccel =
        pigeonYAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());

    // Calculates the jerk in the X and Y directions
    double currentTime = MathSharedStore.getTimestamp();
    double dt = lastUpdateTime == 0.0 ? 0.02 : currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;

    double pigeonXJerk;
    double pigeonYJerk;
    if (dt == 0) {
      pigeonXJerk = 0.0;
      pigeonYJerk = 0.0;
    } else {
      pigeonXJerk = (pigeonFilteredXAccel - pigeonXPrevAccel) / dt;
      pigeonYJerk = (pigeonFilteredYAccel - pigeonYPrevAccel) / dt;
    }

    pigeonXPrevAccel = pigeonFilteredXAccel;
    pigeonYPrevAccel = pigeonFilteredYAccel;

    double pigeonJerk = Math.hypot(pigeonXJerk, pigeonYJerk);

    pigeonGForce = pigeonJerk / 9.81;
    maxGForceDetected = Math.max(maxGForceDetected, pigeonGForce);

    DogLog.log("Imu/Pigeon/XJerk", pigeonXJerk);
    DogLog.log("Imu/Pigeon/YJerk", pigeonYJerk);
    DogLog.log("Imu/Pigeon/HypotAccel", pigeonJerk);
    DogLog.log("Imu/Pigeon/GForce", pigeonGForce);
    DogLog.log("Imu/Pigeon/MaxGForce", maxGForceDetected);
    bumpCrossingTracker.log();
  }

  public boolean collisionDetected() {
    return pigeonGForce > COLLISION_G_FORCE_THRESHOLD.get();
  }
}
