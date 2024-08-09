// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinematics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Class for odometry. Robot code should not use this directly- Instead, use the particular type for
 * your drivetrain (e.g., {@link DifferentialDriveOdometry}). Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from encoders and a
 * gyroscope.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 *
 * @param <T> Wheel positions type.
 */
public class Odometry3D<T extends WheelPositions<T>> {
  private final Kinematics<?, T> m_kinematics;
  private Pose3d m_poseMeters;

  private Rotation3d m_gyroOffset;
  private Rotation3d m_previousAngle;
  private T m_previousWheelPositions;

  /**
   * Constructs an Odometry object.
   *
   * @param kinematics The kinematics of the drivebase.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public Odometry3D(
      Kinematics<?, T> kinematics,
      Rotation3d gyroAngle,
      T wheelPositions,
      Pose3d initialPoseMeters) {
    m_kinematics = kinematics;
    m_poseMeters = initialPoseMeters;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = m_poseMeters.getRotation();
    m_previousWheelPositions = wheelPositions.copy();
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(Rotation3d gyroAngle, T wheelPositions, Pose3d poseMeters) {
    m_poseMeters = poseMeters;
    m_previousAngle = m_poseMeters.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousWheelPositions = wheelPositions.copy();
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose3d getPoseMeters() {
    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method takes in an angle parameter which is used instead of the angular rate
   * that is calculated from forward kinematics, in addition to the current distance measurement at
   * each wheel.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @return The new pose of the robot.
   */
  public Pose3d update(Rotation3d gyroAngle, T wheelPositions) {
    var angle = gyroAngle.plus(m_gyroOffset);

    var twist = m_kinematics.toTwist3d(m_previousWheelPositions, wheelPositions);
    twist.dtheta = angle.minus(m_previousAngle).getRadians();

    var newPose = m_poseMeters.exp(twist);

    m_previousWheelPositions = wheelPositions.copy();
    m_previousAngle = angle;
    m_poseMeters = new Pose3d(newPose.getTranslation(), angle);

    return m_poseMeters;
  }
}
