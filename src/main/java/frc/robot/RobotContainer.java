// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController xboxController = new CommandXboxController(Constants.controller.xboxPort);
  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(Constants.controller.xRateLimit);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Constants.controller.yRateLimit);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.controller.rotRateLimit);

  private final DoubleSupplier getPeriod;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(DoubleSupplier getPeriod) {
    this.getPeriod = getPeriod;
    // Configure the trigger bindings
    SmartDashboard.putNumber("LeftX", xboxController.getLeftX());
    SmartDashboard.putNumber("LeftY", xboxController.getLeftY());
    SmartDashboard.putNumber("RightX", xboxController.getRightX());
    SmartDashboard.putNumber("DriveP", Constants.DriveTrain.driveControllerKp);
    SmartDashboard.putNumber("DriveI", Constants.DriveTrain.driveControllerKi);
    SmartDashboard.putNumber("DriveD", Constants.DriveTrain.driveControllerKd);
    SmartDashboard.getNumber("TurnP", Constants.DriveTrain.turnControllerKp);
    SmartDashboard.getNumber("TurnI", Constants.DriveTrain.turnControllerKi);
    SmartDashboard.getNumber("TurnD", Constants.DriveTrain.turnControllerKd);
    SmartDashboard.putNumber("Turn Target", Constants.DriveTrain.turnTarget);
    //Add Screen Buttons
    SmartDashboard.putData("reset gyro",new InstantCommand(m_swerveDrive::resetHeading));
    //SmartDashboard.putData("Pickup",new Pickup );
    // Configure the trigger and button bindings
    configureBindings();

    m_swerveDrive.setDefaultCommand(new DriveTeleop(
        m_swerveDrive,
        m_xSpeedLimiter,
        m_ySpeedLimiter,
        m_rotLimiter,
        () -> xboxController.getLeftX(),
        () -> xboxController.getLeftY(),
        () -> xboxController.getRightX() / 2,
        false,
        this.getPeriod));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
  }
}
