package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO climber;

  public Climber(ClimberIO climber) {
    this.climber = climber;
  }

  public Command climbUp() {
    return startEnd(() -> climber.setClimberSpeed(0.5), () -> climber.setClimberSpeed(0));
  }

  public Command climbDown() {
    return startEnd(() -> climber.setClimberSpeed(-0.5), () -> climber.setClimberSpeed(0));
  }

  public void Up() {
    climber.setClimberSpeed(1);
  }

  public void Down() {
    climber.setClimberSpeed(-1);
  }

  public void stopClimber() {
    climber.setClimberSpeed(0);
  }

  public double getClimberPosition() {
    return climber.getClimberPosition();
  }

  public void resetClimberPosition() {
    climber.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Climber position",
        Math.round(climber.getClimberPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
  }
}
