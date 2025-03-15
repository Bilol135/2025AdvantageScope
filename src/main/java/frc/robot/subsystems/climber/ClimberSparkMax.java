package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.AlgaeIntakeConfig;

public class ClimberSparkMax implements ClimberIO {
  private final SparkMax pivotMotor;

  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotController;

  public ClimberSparkMax() {
    pivotMotor = new SparkMax(16, MotorType.kBrushless);

    pivotEncoder = pivotMotor.getEncoder();
    pivotController = pivotMotor.getClosedLoopController();
    pivotMotor.configure(
        AlgaeIntakeConfig.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setClimberSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public double getClimberPosition() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void setClimberPosition(double position) {
    pivotController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void reset() {
    pivotEncoder.setPosition(0);
  }
}
