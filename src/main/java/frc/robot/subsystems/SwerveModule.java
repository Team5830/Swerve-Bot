// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import frc.robot.Constants;

public class SwerveModule {
	private Talon angleMotor;
	private Talon speedMotor;
	private PIDController anglePidController;
  private AnalogInput angleEncoder;
  private double setpoint;
  private double m_driveFeedforward;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder,
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int encoderChannel,
      boolean invertEncoder,
      boolean invertMotor,
      double zeroOffset,
      double driveFeedforward) {
    try {
      m_driveFeedforward = driveFeedforward;

      this.angleMotor = new Talon(turningMotorChannel);
      this.speedMotor = new Talon(driveMotorChannel);

      this.angleEncoder = new AnalogInput(encoderChannel);
      anglePidController = new PIDController (0.1,0,0);
      anglePidController.enableContinuousInput(0, 360);
      anglePidController.setTolerance(5,0.5);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating swerve module: " + ex.getMessage(), true);
    }
  }


  public double Angle() {
    // return m_angleEncoder.getPosition()*(180/Math.PI)-180; //(0 to
    // 2PI)*(180/PI)-180
    return angleEncoder.getValue()*(360/5);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var currentAngle = Angle();
    // var encoderRotation = Rotation2d.fromDegrees(angleEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    // SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change
    // directions. This results in smoother driving.
    // state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate feedforward
    ///m_drivePIDController.setFF(m_driveFeedforward.calculate(state.speedMetersPerSecond));

    speedMotor.set(desiredState.speedMetersPerSecond/5);
    anglePidController.setSetpoint(desiredState.angle.getDegrees());
    angleMotor.set(anglePidController.calculate(currentAngle));
    System.out.println("speed");
    System.out.println(desiredState.speedMetersPerSecond/5);
    System.out.println("angle");
    System.out.println(anglePidController.calculate(currentAngle));
    System.out.println("angle setpoint");
    System.out.println(desiredState.angle.getDegrees());
  }

  public void PIDStop() {
    
  }
}
