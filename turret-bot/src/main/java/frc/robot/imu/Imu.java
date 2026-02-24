package frc.robot.imu;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.team581.mechanisms.imu.BaseImuSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.util.scheduling.SubsystemPriority;

public class Imu extends BaseImuSubsystem {
  private static final DoubleSubscriber COLLISION_G_FORCE_THRESHOLD =
      DogLog.tunable("Imu/CollisionGForceThreshold", 2.0);
  private final LinearFilter pigeonXAccelFilter = LinearFilter.movingAverage(10);

  private final LinearFilter pigeonYAccelFilter = LinearFilter.movingAverage(10);

  private double pigeonXPrevAccel = 0.0;
  private double pigeonYPrevAccel = 0.0;
  private double pigeonGForce = 0.0;
  private double maxGForceDetected = Double.NEGATIVE_INFINITY;

  public Imu(SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(SubsystemPriority.IMU, drivetrain);
  }

  @Override
  public void collectInputs() {
    super.collectInputs();

    double pigeonFilteredXAccel =
        pigeonXAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    double pigeonFilteredYAccel =
        pigeonYAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());

    // Calculates the jerk in the X and Y directions
    // Divides by .02 because default loop timing is 20ms
    double pigeonXJerk = (pigeonFilteredXAccel - pigeonXPrevAccel) / 0.02;
    double pigeonYJerk = (pigeonFilteredYAccel - pigeonYPrevAccel) / 0.02;

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
  }

  public boolean collisionDetected() {
    return pigeonGForce > COLLISION_G_FORCE_THRESHOLD.get();
  }
}
