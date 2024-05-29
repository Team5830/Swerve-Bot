// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;


import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private final Translation2d m_frontLeftLocation = new Translation2d(0.438, -0.438);
  private final Translation2d m_frontRightLocation = new Translation2d(0.438, 0.438);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.438, -0.438);
  private final Translation2d m_backRightLocation = new Translation2d(-0.438, 0.438);
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
    
    //ahrs.reset();
  public final SwerveModule m_frontLeft = new SwerveModule(
    5,4,2,
      false, true, 277,
      Constants.DriveTrain.leftFeedforwardStatic
    );
  public final SwerveModule m_frontRight = new SwerveModule(
    3,2,1,
      false, true, 326,
      Constants.DriveTrain.leftFeedforwardStatic
    );
  public final SwerveModule m_backLeft = new SwerveModule(
    0,1,3,
      false, true, 229,
      Constants.DriveTrain.rightFeedforwardStatic
    );
  public final SwerveModule m_backRight = new SwerveModule(
    7,8,0,
      false, true, 222,
      Constants.DriveTrain.rightFeedforwardStatic
    );

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  // private final  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //   m_kinematics, ahrs.getRotation2d(),
  //   new SwerveModulePosition[] {
  //     m_frontLeft.getPosition(),
  //     m_frontRight.getPosition(),
  //     m_backLeft.getPosition(),
  //     m_backRight.getPosition(),
  //   }, Constants.DriveTrain.initialOdometry
  // );

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */

  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative,boolean disableOptimizations) {
      
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(ySpeed, -xSpeed, rot));

   setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveTrain.maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  // public void setModuleStatesBasic(SwerveModuleState[] swerveModuleStates) {
  //       SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveTrain.maxSpeed);
  //   m_frontLeft.setDesiredStateBasic(swerveModuleStates[0]);
  //   m_frontRight.setDesiredStateBasic(swerveModuleStates[1]);
  //   m_backLeft.setDesiredStateBasic(swerveModuleStates[2]);
  //   m_backRight.setDesiredStateBasic(swerveModuleStates[3]);
  // }

  /** Updates the field relative position of the robot. */
  // public void updateOdometry() {
  //   m_poseEstimator.update(
  //       Constants.DriveTrain.invertNavX ? ahrs.getRotation2d().unaryMinus() : ahrs.getRotation2d(),
  //       new SwerveModulePosition[] {
  //           m_frontLeft.getPosition(),
  //           m_frontRight.getPosition(),
  //           m_backLeft.getPosition(),
  //           m_backRight.getPosition()
  //       });

  //   // Also apply vision measurements. We use 0.3 seconds in the past as an example
  //   // -- on a real robot, this must be calculated based either on latency or
  //   // timestamps.
  //   m_poseEstimator.addVisionMeasurement(
  //       ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
  //           m_poseEstimator.getEstimatedPosition()),
  //       Timer.getFPGATimestamp() - 0.3);
  // }

  // public void resetOdometry(Pose2d pose){
  //   m_poseEstimator.resetPosition(Constants.DriveTrain.invertNavX ? ahrs.getRotation2d().unaryMinus() : ahrs.getRotation2d(), 
  //     new SwerveModulePosition[] {
  //           m_frontLeft.getPosition(),
  //           m_frontRight.getPosition(),
  //           m_backLeft.getPosition(),
  //           m_backRight.getPosition()
  //       }, pose);
  // }

  public void stopModules() {
    m_frontLeft.PIDStop();;
    m_frontRight.PIDStop();
    m_backLeft.PIDStop();
    m_backRight.PIDStop();
  }

  // public Pose2d getPose(){
  //   return m_poseEstimator.getEstimatedPosition();
  // }

  // public double getDistance() {
  //   // This most won't work and needs to be changed
  //   return Math.sqrt(Math.pow(m_odometry.getPoseMeters().getX(), 2)+Math.pow(m_odometry.getPoseMeters().getY(), 2));
  // }
  public double getAngle() {
    return ahrs.getAngle();
  }
  public void resetHeading() {
    ahrs.reset();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  @Override
  public void periodic() {
    // //SmartDashboard.putNumber("TurnP", m_frontLeft.getPValue());
    // //SmartDashboard.putNumber("TurnI", m_frontLeft.getIValue());
    // //SmartDashboard.putNumber("TurnD", m_frontLeft.getDValue());
    // SmartDashboard.putNumber("TurnTarget", 0);
    // SmartDashboard.putNumber("FrontLeft Angle", m_frontLeft.Angle());
    // SmartDashboard.putNumber("FrontLeft Position", m_frontLeft.Offset());
    // SmartDashboard.putNumber("FrontRight Angle", m_frontRight.Angle());
    // SmartDashboard.putNumber("FrontRight Position", m_frontRight.Offset());
    // SmartDashboard.putNumber("BackLeft Angle", m_backLeft.Angle());
    // SmartDashboard.putNumber("BackLeft Position", m_backLeft.Offset());
    // SmartDashboard.putNumber("BackRight Angle", m_backRight.Angle());
    // SmartDashboard.putNumber("BackRight Position", m_backRight.Offset());
    // SmartDashboard.putNumber("NAVX Heading", Constants.DriveTrain.invertNavX ? -ahrs.getAngle() : ahrs.getAngle());
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Current Angle", m_frontLeft.Angle());
    // SmartDashboard.putNumber("swerve drive offset", getDriveOffset());
    // SmartDashboard.putNumber("frontLeftDistance",m_frontLeft.getPosition().distanceMeters);
    // SmartDashboard.putNumber("frontLeftAngle",m_frontLeft.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("frontRightDistance",m_frontRight.getPosition().distanceMeters);
    // SmartDashboard.putNumber("frontRightAngle",m_frontRight.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("backLeftDistance",m_backLeft.getPosition().distanceMeters);
    // SmartDashboard.putNumber("backLeftAngle",m_backLeft.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("backRightDistance",m_backRight.getPosition().distanceMeters);
    // SmartDashboard.putNumber("backRightAngle",m_backRight.getPosition().angle.getDegrees());
    // updateOdometry();
  }
}
