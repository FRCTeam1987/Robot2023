// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import static frc.robot.Constants.*;
/**
 * This subsystem models the robot's drivetrain mechanism. It consists of a four MK4 swerve modules,
 * each with two motors and an encoder. It also consists of a Pigeon which is used to measure the
 * robot's rotation.
 */
public class DrivetrainSubsystem extends SubsystemBase {
  
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  // some of this code is from the SDS example code

  private ChassisSpeeds chassisSpeeds;

  private static final String SUBSYSTEM_NAME = "Drivetrain";

  private final SwerveDrivePoseEstimator poseEstimator;

  private Pigeon2 gyro;

  /** Constructs a new DrivetrainSubsystem object. */
  public DrivetrainSubsystem(
      int gyroID,
      SwerveModule flModule,
      SwerveModule frModule,
      SwerveModule blModule,
      SwerveModule brModule) {
    this.gyro = new Pigeon2(gyroID, CAN_BUS_NAME);
    ;
    frontLeftModule = flModule;
    frontRightModule = frModule;
   backLeftModule = blModule;
    backRightModule = brModule;
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            KINEMATICS, // kinematics object for drivetrain
            Rotation2d.fromDegrees(0.0), // current rotation
            new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()}, // currrent swerve module positions
            new Pose2d() // starting pose
            );

    this.zeroGyroscope();


    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);



    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
    // tabMain.addBoolean("X-Stance On?", this::isXstance);
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment beteween the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()}, new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0.0)));
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive. This method should always be invoked instead of obtaining the yaw directly from the
   * Pigeon as the local offset needs to be added. If the gyro is not connected, the rotation from
   * the estimated pose is returned.
   *
   * @return the rotation of the robot
   */
  private Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field to the lower left corner (i.e., the corner of the field to
   * the driver's right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @return the pose of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) { 
    this.chassisSpeeds = speeds;
  }

  public void resetPose() {
    setPose(new Pose2d());
  }

  public void setPose(final Pose2d pose) {
    poseEstimator.resetPosition(new Rotation2d(gyro.getYaw()), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()}, pose);
  }

  /**
   * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
   * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
   * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
   * right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param state the specified PathPlanner state to which is set the odometry
   */
  // public void resetOdometry(PathPlannerState state) {
  //   setGyroOffset(state.holonomicRotation.getDegrees());

  //   for (int i = 0; i < 4; i++) {
  //     swerveModulePositions[i] = swerveModules[i].getPosition();
  //   }

  //   estimatedPoseWithoutGyro =
  //       new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
  //   poseEstimator.resetPosition(
  //       this.getRotation(),
  //       swerveModulePositions,
  //       new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
  // }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities may be specified from either the robot's frame of
   * reference of the field's frame of reference. In the robot's frame of reference, the positive x
   * direction is forward; the positive y direction, left; position rotation, CCW. In the field
   * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
   * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
   * direction.
   *
   * <p>If the drive mode is XSTANCE, the robot will ignore the specified velocities and turn the
   * swerve modules into the x-stance orientation.
   *
   * <p>If the drive mode is CHARACTERIZATION, the robot will ignore the specified velocities and
   * run the characterization routine.
   *
   * @param translationXSupplier the desired velocity in the x direction (m/s)
   * @param translationYSupplier the desired velocity in the y direction (m/s)
   * @param rotationSupplier the desired rotational velcoity (rad/s)
   */
 
   public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
}


public void drive(double x, double y, double rot) {
  this.chassisSpeeds = new ChassisSpeeds(x,y,rot);
}

  /**
   * Stops the motion of the robot. Since the motors are in break mode, the robot will stop soon
   * after this method is invoked.
   */
  public void stop() {
    drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  /**
   * This method is invoked each iteration of the scheduler. Typically, when using a command-based
   * model, subsystems don't override the periodic method. However, the drivetrain needs to
   * continually update the odometry of the robot, update and log the gyro and swerve module inputs,
   * update brake mode, and update the tunable values.
   */
  @Override
  public void periodic() {
      poseEstimator.update(
              Rotation2d.fromDegrees(gyro.getYaw()),
              new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
      );

      SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }

  /**
   * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
   * stopped moving, and brake mode is enabled, disable it.
   */

  /**
   * Returns true if field relative mode is enabled
   *
   * @return true if field relative mode is enabled
   */

  /**
   * Enables field-relative mode. When enabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the field.
   */
  /**
   * Disables field-relative mode. When disabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the robot.
   */
  /**
   * Sets the swerve modules in the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
   * useful when shooting.
   */
  // public void setXStance() {
  //   chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  //   SwerveModuleState[] states =
  //       Drivetrain.KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerGravity);
  //   states[0].angle =
  //       new Rotation2d(
  //           Math.PI / 2 - Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
  //   states[1].angle =
  //       new Rotation2d(
  //           Math.PI / 2 + Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
  //   states[2].angle =
  //       new Rotation2d(
  //           Math.PI / 2 + Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
  //   states[3].angle =
  //       new Rotation2d(
  //           3.0 / 2.0 * Math.PI
  //               - Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
  //   for (SwerveModule swerveModule : swerveModules) {
  //     swerveModule.set(0, states[swerveModule.getModuleNumber()].angle.getDegrees());
  //   }
  // }


  /**
   * Returns the desired velocity of the drivetrain in the x direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the x direction (units of m/s)
   */
  public double getVelocityX() {
    return chassisSpeeds.vxMetersPerSecond;
  }

  /**
   * Returns the desired velocity of the drivetrain in the y direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the y direction (units of m/s)
   */
  public double getVelocityY() {
    return chassisSpeeds.vyMetersPerSecond;
  }

  /**
   * Puts the drivetrain into the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
   * useful when shooting. The robot cannot be driven until x-stance is disabled.


  /**
   * Returns true if the robot is in the x-stance orientation.
   *
   * @return true if the robot is in the x-stance orientation
   */

  /**
   * Returns the PID controller used to control the robot's x position during autonomous.
   *
   * @return the PID controller used to control the robot's x position during autonomous
   */


  /** Runs forwards at the commanded voltage. */

  /** Returns the average drive velocity in meters/sec. */


  public double getPitch() {
    return gyro.getPitch();
  }

  public Rotation2d getYaw() {
    return new Rotation2d(gyro.getYaw());
  }

}
