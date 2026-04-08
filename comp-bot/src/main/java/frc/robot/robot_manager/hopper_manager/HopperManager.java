package frc.robot.robot_manager.hopper_manager;

import com.ctre.phoenix6.hardware.CANrange;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  public final Deploy deploy;
  public final Intake intake;
  public final Conveyor conveyor;
  public final Feeder feeder;
  public final CANrange hopperCANRange;
  public final DigitalInput towerSensor;

  private final Debouncer towerSensorDebouncer = new Debouncer(0.25, DebounceType.kFalling);
  private boolean towerSensorDebounced = false;

  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;
  private boolean towerSensorRaw = false;

  private final LinearFilter hopperFilter = LinearFilter.movingAverage(50);

  private double hopperDistance = 0.0;
  private double filteredDistance = 0.0;
  private double previousCanRangeDistance = 0.0;
  public static final double HIGH_CAPACITY_THRESHOLD = 5;
  public static final double MEDIUM_CAPACITY_THRESHOLD = 10;

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;

  private final Timer canRangeUpdateTimer = new Timer();

  public HopperManager(
      Deploy deploy,
      Intake intake,
      Conveyor conveyor,
      Feeder feeder,
      CANrange hopperCANRange,
      DigitalInput towerSensor) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE_DEPLOYED);
    this.deploy = deploy;
    this.intake = intake;
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.hopperCANRange = hopperCANRange;
    this.towerSensor = towerSensor;

    hopperCANRange.getConfigurator().apply(HopperManagerConfig.CAN_RANGE_CONFIG);
    canRangeUpdateTimer.start();
  }

  @Override
  protected HopperState getNextState(HopperState currentState) {
    return switch (getState()) {
      case IDLE_DEPLOYED, IDLE_STOWED, INTAKING, EJECTING -> {
        yield resolveIdleState();
      }
      default -> currentState;
    };
  }

  private boolean shouldFillBalls() {
    if (towerSensorDebounced) {
      // The sensor in the tower shows we are holding fuel, so we can't fill anymore
      return false;
    }
    if (!deploy.isFullyExtended()) {
      return false;
    }
    if (DSOptions.USE_CANRANGE.get()) {
      // If we are using the hopper CANrange, we can start filling the tower once the hopper is
      // starting ot fill up
      return hopperCapacity == HopperCapacity.MEDIUM || hopperCapacity == HopperCapacity.HIGH;
    }

    // Otherwise, we fallback to running once we've been intaking for a few seconds
    return intake.hasBeenIntaking();
  }

  /** Sets conveyor and feeder to ball filling if conditions are met, otherwise idles them. */
  private void smartBallFillRequest() {
    if (shouldFillBalls()) {
      conveyor.ballFillingRequest();
      feeder.ballFillingRequest();
    } else {
      conveyor.idleRequest();
      feeder.idleRequest();
    }
  }

  private void smartIntakeBallFillRequest() {
    if (shouldFillBalls()) {
      conveyor.ballFillingRequest();
      feeder.ballFillingRequest();
    } else {
      if (towerSensorDebounced) {
        conveyor.idleRequest();
        feeder.idleRequest();
      } else {
        conveyor.intakeRequest();
        feeder.intakeRequest();
      }
    }
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case IDLE_DEPLOYED -> {
        intake.idleRequest();
        smartBallFillRequest();
      }
      case IDLE_STOWED -> {
        deploy.stowRequest();
        intake.idleRequest();
        smartBallFillRequest();
      }
      case INTAKING -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        smartIntakeBallFillRequest();
      }
      case EJECTING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.ejectRequest();
        feeder.idleRequest();
      }
      case UNJAMMING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case SHOOT, SHOOT_AND_INTAKE -> {
        // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
        if (deploy.getState() != DeployState.HOPPER_COMPACTION_IN) {
          deploy.intakeRequest();
        }
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
    }
  }

  @Override
  protected void whileInState(HopperState state) {
    if (state.canBallFill) {
      if (state == HopperState.INTAKING) {
        smartIntakeBallFillRequest();
      } else {

        smartBallFillRequest();
      }
    }

    switch (state) {
      default -> {}
      case SHOOT -> {
        if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          deploy.hopperCompactionRequest();
        } else {
          deploy.waitHopperCompactionRequest();
        }
      }
      case IDLE_DEPLOYED -> {
        if (timeout(0.5)) {
          deploy.intakeRequest();
        }
      }
    }

    if (previousCanRangeDistance != hopperDistance) {
      canRangeUpdateTimer.reset();
    }
    previousCanRangeDistance = hopperDistance;

    if (canRangeUpdateTimer.hasElapsed(3.0) && DSOptions.USE_CANRANGE.get()) {
      DogLog.logFault("CANrange distance not updating", AlertType.kError);
    } else {
      DogLog.clearFault("CANrange distance not updating");
    }
    DogLog.log("HopperManager/BallFilling", shouldFillBalls() && state.canBallFill);
    DogLog.log("HopperManager/DriverWantsEject", driverWantsEject);
    DogLog.log("HopperManager/DriverWantsIntake", driverWantsIntake);
    DogLog.log("HopperManager/OperatorWantsStow", operatorWantsStow);
    DogLog.log("HopperManager/FilteredHopperDistance", filteredDistance);
    DogLog.log("HopperManager/HopperCapacity", hopperCapacity);
    DogLog.log("HopperManager/TowerSensor", towerSensorRaw);
  }

  private void setState(HopperState newState) {
    setStateFromRequest(newState);
  }

  private HopperState resolveIdleState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake) {
      return HopperState.INTAKING;
    }

    if (operatorWantsStow) {
      return HopperState.IDLE_STOWED;
    }

    return HopperState.IDLE_DEPLOYED;
  }

  private HopperState resolveScoreState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake && deploy.getState() != DeployState.HOPPER_COMPACTION_IN) {
      return HopperState.SHOOT_AND_INTAKE;
    }

    return HopperState.SHOOT;
  }

  public void scoreRequest() {
    setState(resolveScoreState());
  }

  public boolean isShooting() {
    if (RobotBase.isSimulation()) {
      return !timeout(1.5);
    }
    return towerSensorDebounced;
  }

  public void idleRequest() {
    setState(resolveIdleState());
  }

  public void unjamRequest() {
    setState(HopperState.UNJAMMING);
  }

  public void setDriverWantsEject(boolean wantsEject) {
    driverWantsEject = wantsEject;
  }

  public void setDriverWantsIntake(boolean wantsIntake) {
    driverWantsIntake = wantsIntake;
  }

  public void setOperatorWantsStow(boolean wantsStow) {
    operatorWantsStow = wantsStow;
  }

  @Override
  protected void collectInputs() {
    towerSensorRaw = towerSensor.get();
    towerSensorDebounced = towerSensorDebouncer.calculate(towerSensorRaw);
    if (DSOptions.USE_CANRANGE.get()) {
      hopperDistance = Units.metersToInches(hopperCANRange.getDistance().getValueAsDouble());
      filteredDistance = hopperFilter.calculate(hopperDistance);
    }
    if (RobotBase.isSimulation()) {
      hopperDistance = 20;
      towerSensorRaw =
          switch (getState()) {
            case IDLE_DEPLOYED, IDLE_STOWED, INTAKING -> !timeout(5);
            default -> false;
          };
    }
    filteredDistance = hopperFilter.calculate(hopperDistance);

    if (filteredDistance <= HIGH_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.HIGH;
    } else if (filteredDistance <= MEDIUM_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.MEDIUM;
    } else {
      hopperCapacity = HopperCapacity.LOW;
    }
  }
}
