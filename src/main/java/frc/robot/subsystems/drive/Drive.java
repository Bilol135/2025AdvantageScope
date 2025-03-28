// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Field2d m_field = new Field2d();
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // private final VisionIOLimelight visionIO;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  // private SwerveDrivePoseEstimator poseEstimator =
  //     new SwerveDrivePoseEstimator(
  //         kinematics,
  //         rawGyroRotation,
  //         lastModulePositions,
  //         new Pose2d(),
  //         VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
  //         VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    SmartDashboard.putData("Field", m_field);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    RobotConfig pathPlannerConfig = ppConfig;

    try {
      pathPlannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        pathPlannerConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      // poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // if (DriveConstants.useVision) {
    //   poseEstimator.update(gyroIO.getRotation2D(), getModulePositions());
    //   updateOdometry(poseEstimator, gyroIO);
    // }

    // if (DriveConstants.useVision) {
    //   if (RobotBase.isSimulation()) {
    //     visionIO.updateInputs(visionInputs, getPose(), odometry.getPoseMeters());
    //   } else {
    //     visionIO.updateInputs(visionInputs, getPose(), rawGyroRotation);
    //   }
    //   Logger.processInputs("Vision", visionInputs);
    //   if (visionInputs.hasEstimate) {
    //     List<Matrix<N3, N1>> stdDeviations = visionIO.getStdArray(visionInputs, getPose());

    //     for (int i = 0; i < visionInputs.estimate.length; i++) {
    //       if (visionInputs.estimate[i].equals(new Pose2d())) continue; // Camera i has no
    // estimate
    //       else if (stdDeviations.size() <= i || visionInputs.timestampArray.length <= i)
    //         continue; // Avoids index out of bounds exceptions
    //       else {
    //         poseEstimator.addVisionMeasurement(
    //             visionInputs.estimate[i], visionInputs.timestampArray[i], stdDeviations.get(i));
    //       }
    //     }
    //   }
    // }

    SmartDashboard.putNumber("Gyro Yaw", getRotation().getDegrees());
    SmartDashboard.putNumber("Pose Angle", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("SlowMode", DriveCommands.getSlowMode());

    m_field.setRobotPose(odometry.getPoseMeters());

    // Elastic setup
    // SmartDashboard.putData(
    //     "Swerve Drive",
    //     new Sendable() {
    //       @Override
    //       public void initSendable(SendableBuilder builder) {
    //         builder.setSmartDashboardType("SwerveDrive");
    //         // FL, FR, BL, BR
    //         builder.addDoubleProperty(
    //             "Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
    //         builder.addDoubleProperty("Front Left Velocity", () -> modules[0].getVelocity(),
    // null);

    //         builder.addDoubleProperty(
    //             "Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
    //         builder.addDoubleProperty("Front Right Velocity", () -> modules[1].getVelocity(),
    // null);

    //         builder.addDoubleProperty(
    //             "Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
    //         builder.addDoubleProperty("Back Left Velocity", () -> modules[2].getVelocity(),
    // null);

    //         builder.addDoubleProperty(
    //             "Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
    //         builder.addDoubleProperty("Back Right Velocity", () -> modules[3].getVelocity(),
    // null);

    //         builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
    //       }
    //     });
  }

  public void updateOdometry(SwerveDrivePoseEstimator poseEstimator, GyroIO gyroIO) {

    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdateRight = false;
    boolean doRejectUpdateLeft = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdateRight = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdateRight = true;
        }
      }
      if (mt1.tagCount == 0) {
        doRejectUpdateRight = true;
      }

      if (!doRejectUpdateRight) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation(
          "limelight-right",
          poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0);
      LimelightHelpers.PoseEstimate mt2Right =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
      LimelightHelpers.SetRobotOrientation(
          "limelight-left",
          poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0);
      LimelightHelpers.PoseEstimate mt2Left =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      if (Math.abs(gyroIO.getRate())
          > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
      // updates
      {
        doRejectUpdateRight = true;
        doRejectUpdateLeft = true;
      }
      if (mt2Right.tagCount == 0) {
        doRejectUpdateRight = true;
      }
      if (mt2Left.tagCount == 0) {
        doRejectUpdateLeft = true;
      }
      if (!doRejectUpdateRight) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        poseEstimator.addVisionMeasurement(mt2Right.pose, mt2Right.timestampSeconds);
      }
      if (!doRejectUpdateLeft) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        poseEstimator.addVisionMeasurement(mt2Left.pose, mt2Left.timestampSeconds);
      }
    }
  }

  /**
   * A command that automatically aligns to the closest reef position
   *
   * @return
   */
  public Command alignToReef() {
    return new AlignToPose(
        this,
        () -> {
          Pose2d[] reefPoses = FieldConstants.ReefScoringPositions;
          Pose2d currentPose = getPose();

          int closestPose = 0;
          double closestDistance = Double.MAX_VALUE;

          for (int i = 0; i < 12; i++) {
            Transform2d currentToTarget = AllianceFlipUtil.apply(reefPoses[i]).minus(currentPose);
            double distance = currentToTarget.getTranslation().getNorm();

            if (distance < closestDistance) {
              closestPose = i;
              closestDistance = distance;
            }
          }

          return AllianceFlipUtil.apply(reefPoses[closestPose]);
        },
        false);
  }

  /**
   * A command that automatically aligns to the closest reef position
   *
   * @return
   */
  public Command alignToReefAuto() {
    return new AlignToPose(
        this,
        () -> {
          Pose2d[] reefPoses = FieldConstants.ReefScoringPositions;
          Pose2d currentPose = getPose();

          int closestPose = 0;
          double closestDistance = Double.MAX_VALUE;

          for (int i = 0; i < 12; i++) {
            Transform2d currentToTarget = AllianceFlipUtil.apply(reefPoses[i]).minus(currentPose);
            double distance = currentToTarget.getTranslation().getNorm();

            if (distance < closestDistance) {
              closestPose = i;
              closestDistance = distance;
            }
          }

          return AllianceFlipUtil.apply(reefPoses[closestPose]);
        },
        true);
  }

  public void applySlowMode() {
    speedIndex = kslowModeConstant;
  }

  public void resetSpeedIndex() {
    speedIndex = 1;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  // public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
  //   poseEstimator.addVisionMeasurement(visionPose, timestamp);
  // }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return gyroIO.getRotation2D();
  }

  public double getRobotYaw() {
    return gyroIO.getYawAngle();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    gyroIO.setYawAngle(pose.getRotation().getDegrees());
    rawGyroRotation = pose.getRotation();

    // Yes I know it says that you don't need to reset the gyro rotation, but it tweaks out if you
    // don't
    // poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  // /** Adds a new timestamped vision measurement. */
  // public void addVisionMeasurement(
  //     Pose2d visionRobotPoseMeters,
  //     double timestampSeconds,
  //     Matrix<N3, N1> visionMeasurementStdDevs) {
  //   poseEstimator.addVisionMeasurement(
  //       visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  // }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}
