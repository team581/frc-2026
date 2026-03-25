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

  private final Debouncer towerSensorDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private boolean towerSensorDebounced = false;

  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;
  private boolean towerSensorRaw = false;

  private final LinearFilter hopperFilter = LinearFilter.movingAverage(5);

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

  private boolean shouldFillBalls() {
    return ((intake.hasBeenIntaking() && !DSOptions.USE_CANRANGE.get())
            || (hopperCapacity == HopperCapacity.MEDIUM || hopperCapacity == HopperCapacity.HIGH))
        && !towerSensorRaw;
  }

  @Override
  protected HopperState getNextState(HopperState currentState) {
    return switch (currentState) {
      case SHOOT, SHOOT_AND_INTAKE, EJECTING -> currentState;
      case IDLE_DEPLOYED, IDLE_STOWED -> {
        if (shouldFillBalls()) {
          yield HopperState.BALL_FILLING;
        }
        yield currentState;
      }
      case INTAKING -> {
        if (shouldFillBalls()) {
          yield HopperState.BALL_FILLING_INTAKING;
        }
        yield currentState;
      }
      case BALL_FILLING -> {
        if (towerSensorDebounced) {
          yield HopperState.IDLE_DEPLOYED;
        }
        if (driverWantsIntake) {
          yield HopperState.BALL_FILLING_INTAKING;
        }
        yield currentState;
      }
      case BALL_FILLING_INTAKING -> {
        if (towerSensorDebounced) {
          yield HopperState.IDLE_DEPLOYED;
        }
        if (!driverWantsIntake) {
          yield HopperState.BALL_FILLING;
        }
        yield currentState;
      }
      case REHOME_DEPLOY -> deploy.getState().isHoming() ? currentState : HopperState.IDLE_DEPLOYED;
    };
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case IDLE_DEPLOYED -> {
        deploy.intakeRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
      case IDLE_STOWED -> {
        deploy.stowRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
      case INTAKING -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
      case EJECTING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.ejectRequest();
        feeder.idleRequest();
      }
      case BALL_FILLING -> {
        deploy.intakeRequest();
        intake.idleRequest();
        conveyor.ballFillingRequest();
        feeder.ballFillingRequest();
      }
      case BALL_FILLING_INTAKING -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.ballFillingRequest();
        feeder.ballFillingRequest();
      }
      case SHOOT -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case SHOOT_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case REHOME_DEPLOY -> {
        deploy.homingRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
    }
  }

  @Override
  protected void whileInState(HopperState state) {
    switch (state) {
      default -> {}
      case SHOOT -> {
        if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          deploy.hopperCompactionRequest();
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
    DogLog.log("HopperManager/State", getState());
    DogLog.log("HopperManager/DriverWantsEject", driverWantsEject);
    DogLog.log("HopperManager/DriverWantsIntake", driverWantsIntake);
    DogLog.log("HopperManager/OperatorWantsStow", operatorWantsStow);
    DogLog.log("HopperManager/FilteredHopperDistance", filteredDistance);
    DogLog.log("HopperManager/HopperCapacity", hopperCapacity);
    DogLog.log("HopperManager/TowerSensor", towerSensorRaw);
  }

  private void setState(HopperState newState) {
    switch (deploy.getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(newState);
    }
  }

  private boolean isBallFilling() {
    return getState() == HopperState.BALL_FILLING
        || getState() == HopperState.BALL_FILLING_INTAKING;
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

    if (driverWantsIntake) {
      return HopperState.SHOOT_AND_INTAKE;
    }

    return HopperState.SHOOT;
  }

  public void scoreRequest() {
    setState(resolveScoreState());
  }

  public void idleRequest() {
    if (isBallFilling()) {
      return;
    }
    setState(resolveIdleState());
  }

  public void homeDeployRequest() {
    setStateFromRequest(HopperState.REHOME_DEPLOY);
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
            case BALL_FILLING, BALL_FILLING_INTAKING -> timeout(1.5);
            case IDLE_DEPLOYED, IDLE_STOWED -> !timeout(5);
            default -> false;
          };
    }
    filteredDistance = hopperFilter.calculate(hopperDistance);

    if (filteredDistance <= MEDIUM_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.MEDIUM;
    } else if (filteredDistance <= HIGH_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.HIGH;
    } else {
      hopperCapacity = HopperCapacity.LOW;
    }
  }
}
