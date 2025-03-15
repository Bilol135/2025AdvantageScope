package frc.robot.subsystems.climber;

public interface ClimberIO {
  /**
   * Sets the speed of the climber motor.
   *
   * @param speed The speed to set the climber motor to.
   */
  public void setClimberSpeed(double speed);

  /**
   * Gets the encoder value of the climber motor.
   *
   * @return The encoder value of the climber motor.
   */
  public double getClimberPosition();

  public void setClimberPosition(double position);

  /** Resets the encoder value of the climber motor. */
  public void reset();
}
