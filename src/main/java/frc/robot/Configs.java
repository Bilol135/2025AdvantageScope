package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public final class Configs {
  public static final class ElevatorConfig {
    public static final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    public static final SoftLimitConfig elevatorSoftLimit = new SoftLimitConfig();

    static {
      leftMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      leftMotorConfig.closedLoop.pid(
          ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
      rightMotorConfig.apply(leftMotorConfig).follow(ElevatorConstants.kElevatorLeftMotorID, false);
    }
  }

  public static final class ClimberConfig {
    public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    public static final SoftLimitConfig pivotSoftLimit = new SoftLimitConfig();

    static {
      pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      pivotConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.1, 0.0, 0.0)
          .outputRange(-1.0, 1.0);
      pivotSoftLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(4.5f)
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(0.0f);
    }
  }

  public static final class AlgaeIntakeConfig {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    public static final SoftLimitConfig pivotSoftLimit = new SoftLimitConfig();

    static {
      intakeConfig.idleMode(IdleMode.kBrake);
      pivotConfig.idleMode(IdleMode.kBrake);
      pivotConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(AlgaeIntakeConstants.kP, AlgaeIntakeConstants.kI, AlgaeIntakeConstants.kD)
          .outputRange(AlgaeIntakeConstants.kMinOutput, AlgaeIntakeConstants.kMaxOutput);
      /*pivotSoftLimit
              .forwardSoftLimitEnabled(true)
              .forwardSoftLimit((float)AlgaeIntakeConstants.kPivotMaxPosition)
              .reverseSoftLimitEnabled(true)
              .reverseSoftLimit((float)AlgaeIntakeConstants.kPivotMinPosition);
      */
    }
  }

  public static final class CoralIntakeConfig {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    public static final SoftLimitConfig pivotSoftLimit = new SoftLimitConfig();

    static {
      pivotConfig.idleMode(IdleMode.kBrake);
      pivotConfig
          .closedLoop
          .pid(CoralIntakeConstants.kP, CoralIntakeConstants.kI, CoralIntakeConstants.kD)
          .velocityFF(CoralIntakeConstants.kFF)
          .outputRange(CoralIntakeConstants.kMinOutput, CoralIntakeConstants.kMaxOutput);
      pivotSoftLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit((float) CoralIntakeConstants.kPivotMaxPosition)
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit((float) CoralIntakeConstants.kPivotMinPosition);
      intakeConfig.idleMode(IdleMode.kBrake);
    }
  }
}
