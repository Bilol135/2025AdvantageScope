package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double setpointMeters = 0.0;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public boolean limitSwitchPressed = false;

    // arrays are used because we have multiple motors
    public double[] motorTemperature = new double[] {};
    public double[] motorCurrent = new double[] {};
  }

  /**
   * Moves the elevator motors up.
   *
   * @param speed The speed to set the motors to.
   */
  public void elevatorUp(double speed);

  /**
   * Moves the elevator motors down.
   *
   * @param speed The speed to set the motors to.
   */
  public void elevatorDown(double speed);

  /**
   * Sets the elevator motors to a specific position.
   *
   * @param position The position to set the motors to.
   */
  public void setElevatorPosition(double position);

  /** Stops the elevator motors. */
  public void elevatorStop();

  /**
   * Gets the encoder value of the elevator motors.
   *
   * @return The encoder value of the elevator motors.
   */
  public double getElevatorPosition();

  /** Resets the encoder value of the elevator motors. */
  public void resetElevator();

  public double getLvoltage();

  public double getRvoltage();

  public double getLcurrent();

  public double getRcurrent();

  public double getLoutput();

  public double getRoutput();

  public double getLtemp();

  public double getRtemp();

  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Returns the setpoint of the elevator */
  public default double getSetpoint() {
    return 0.0;
  }

  /**
   * Tells the elevator to start moving towards a setpoint (height)
   *
   * @param setpoint the target height, in meters, the elevator is trying to go to
   */
  public default void setSetpoint(double setpoint) {}

  /** Runs PID and feedforward to move towards setpoint */
  public default void goToSetpoint() {}

  /** Returns the elevator's instantaneous height */
  public default double getPosition() {
    return 0.0;
  }

  /** returns true if the elevator is at the setpoint */
  public default boolean atSetpoint() {
    return false;
  }

  /** Returns the velocity of the elevator in meters per second */
  public default double getVelocity() {
    return 0.0;
  }

  /**
   * Sets the elevator's voltage
   *
   * @param voltage voltage fed into motor from -12 to 12
   */
  public default void setVoltage(double voltage) {}

  /**
   * Sets the brakemode of both motors
   *
   * @param brakeEnabled true = brake, false = coast
   */
  public default void setBrakeMode(boolean brakeEnabled) {}

  /**
   * Returns the status of the limit switch
   *
   * @return true if limit switch is pressed, false otherwise
   */
  public default boolean isLimitSwitchPressed() {
    return false;
  }

  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default double getP() {
    return 0.0;
  }
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default double getI() {
    return 0.0;
  }
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default double getD() {
    return 0.0;
  }

  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default double getkS() {
    return 0.0;
  }

  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default double getkG() {
    return 0.0;
  }

  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default double getkV() {
    return 0.0;
  }

  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default double getkA() {
    return 0.0;
  }

  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default void setP(double kP) {}
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default void setI(double kI) {}
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default void setD(double kD) {}
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default void setkS(double kS) {}
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default void setkG(double kG) {}
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default void setkV(double kV) {}
  /**
   * Only use individual getters and setters for spontaneous changes during runtime. All PID
   * constants are automatically set when the robot starts up
   */
  public default void setkA(double kA) {}
}
