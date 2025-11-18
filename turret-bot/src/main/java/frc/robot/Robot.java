package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.ControllerHelpers;
import com.team581.math.MathHelpers;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.pure_pursuit.PurePursuitPathTracker;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.autos.Autos;
import frc.robot.generated.BuildConstants;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.SwerveSubsystem;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem(hardware.drivetrain);

  private final LocalizationSubsystem localization =
      new LocalizationSubsystem(swerve, hardware.drivetrain);

  private final RobotManager robotManager = new RobotManager(localization, swerve);

  private final Trailblazer trailblazer =
      new Trailblazer(
          swerve,
          localization,
          new PurePursuitPathTracker(true, true),
          new PidPathFollower(
              new PIDController(3.7, 0, 0),
              new PIDController(3.7, 0, 0),
              new PIDController(3.0, 0, 0)));

  @SuppressWarnings("unused") // Registers itself as a subsystem
  private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {
    logMetadata(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY);

    finalizeInit();
  }

  @Override
  protected void configureBindings() {}

  @Override
  public void teleopPeriodic() {
    var leftX = hardware.driverController.getLeftX();
    var leftY = -hardware.driverController.getLeftY();
    var rightX = hardware.driverController.getRightX();

    DogLog.log("DEBUG/TeleopInputs/RightX", rightX);

    var translationMagnitude = ControllerHelpers.getJoystickMagnitude(leftX, leftY, 2);
    var rotationMagnitude =
        Math.copySign(ControllerHelpers.getJoystickMagnitude(rightX, 0, 5), rightX);

    DogLog.log("DEBUG/TeleopInputs/RotationMagnitude", rotationMagnitude);

    swerve.setTeleopInputs(
        translationMagnitude, MathHelpers.rotation2d(leftX, leftY), rotationMagnitude);

    if (hardware.driverController.getBackButtonPressed()) {
      localization.zeroGyro();
    }
  }
}
