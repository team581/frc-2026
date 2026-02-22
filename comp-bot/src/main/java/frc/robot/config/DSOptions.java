package frc.robot.config;

import com.team581.config.DSOption;
import edu.wpi.first.networktables.BooleanSubscriber;

public final class DSOptions {
	public static final BooleanSubscriber USE_TAG_LIMELIGHTS = DSOption.of("UseTagLimelights", true);
	public static final BooleanSubscriber USE_HUB_STATE = DSOption.of("UseHubState", false);<<<<<<<HEAD
	public static final BooleanSubscriber AUTO_SCORE = DSOption.of("AutoScore", false);=======>>>>>>>
	adcc49f0 (Swerve added swerve assist to swerve compbot collect inputs)
	public static final BooleanSubscriber USE_SWERVE_ASSIST = DSOption.of("UseSwerveAssist", true);
	public static final BooleanSubscriber USE_CANRANGE = DSOption.of("UseCANRange", false);
	public static final BooleanSubscriber USE_TURRET = DSOption.of("UseTurret", true);

	// TODO: This should be true when the CANrange is actually physically on the robot
	private DSOptions() {
	}
}
