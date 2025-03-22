package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorConstants {
  public static final double kElevatorUpSpeed = 0.5;
  public static final double kElevatorDownSpeed = -0.01;

  public static final double kMaxVelocity = 2.0;
  public static final double kMaxAcceleration = 2.0;

  public static final int kElevatorLeftMotorID = 10;
  public static final int kElevatorRightMotorID = 11;

  public static final double kElevatorMaxHeight = 30;
  public static final double kElevatorMinHeight = 0.0;

  public static final double kElevatorLevel1 = 0.0;
  public static final double kElevatorLevel2 = 4.2;
  public static final double kElevatorLevel3 = 13.0;
  public static final double kElevatorLevel4 = 28.5;

  public static final double kP = 0.07;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = new ElevatorFeedforward(0.1, 1.3, 0.6, 0.05).calculate(0);
  public static final double kMinOutput = -1.0;
  public static final double kMaxOutput = 1.0;

  public static final double kElevatorVoltage = 2; // for static routine testing
  // output up = 1.1
  // output down = 1
  public static double kSproketRadiusMeters = 0.0508; // 2 in
  public static double kGearRatio = 1.0 / 6.0; //  6:1 gearbox ratio
  public static double kElevatorMass = 50;

  public static class ElevatorSimConstants {
    public static final double[] kElevatorSimPID = {300, 0, 0};
    public static final double[] kElevatorSimFF = {1.4841E-06, 0.12642, 17.748, 1.5689E-05};
    // Converting encoder steps to meters
    public static final double kElevatorGearRatioSim = 20;
    public static final double kElevatorInitalHeight = 0.2;

    // 4096 pulses/revolution
    public static final double kEncoderDisPePurlse =
        2 * Math.PI / 4096 * kSproketRadiusMeters * kGearRatio;
    // public static final int kMotorPort = 2;

  }
}
