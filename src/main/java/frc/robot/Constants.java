package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveTrain {
    public static final double maxSpeed = 3.0; // 4 meters per second
    public static final double maxAcceleration = 3.0;
    public static final double maxAngularVelocity = 15; // revolutions per second
    public static final double maxAngularAcceleration = 20; // revolutions per second squared
    public static final double AngleTolerance = 5;
    public static final double turnTarget = 90;
    public static final double lAlignTolerance = 0.1;
    public static final double lMaxAlignSpeed = 1.0;
    public static final double wheelCircumferenceInches = 12.5;
    public static final double driveGearRatio = 6.55;
    // PIDs
    public static final double driveControllerKp = 0.11;
    public static final double driveControllerKi = 0;
    public static final double driveControllerKd = 0;
    public static final double turnControllerKp = .02;
    public static final double turnControllerKi = 0;
    public static final double turnControllerKd = 0.002;

    // Feedforward gains
    public static final double leftFeedforwardStatic = 0.40179;
    public static final double leftFeedforwardVelocity = 2.6343;
    public static final double leftFeedforwardAcceleration = 0.51816;
    public static final double rightFeedforwardStatic = 0.36268;
    public static final double rightFeedforwardVelocity = 2.7308;
    public static final double rightFeedforwardAcceleration = 0.70894;

    // Channels
    public static final int frontLeftDriveChannel = 1;
    public static final int frontLeftTurnChannel = 2;
    public static final int frontRightDriveChannel = 4;
    public static final int frontRightTurnChannel = 3;
    public static final int backLeftDriveChannel = 7;
    public static final int backLeftTurnChannel = 8;
    public static final int backRightDriveChannel = 5;
    public static final int backRightTurnChannel = 6;

    // Odometry
    public static final boolean invertNavX = false;
    public static final Pose2d initialOdometry = new Pose2d(5.0, 13.5, new Rotation2d());
  }

  public static final class controller {
    public static final int xboxPort = 0;
    public static final int flyskyPort = 1;

    public static final double xRateLimit = 20;
    public static final double yRateLimit = 20;
    public static final double rotRateLimit = 2;

    public static final double climberAxesThreshold = 0.4;
    public static final double climberAxesMultiplier = 10;
  }

  public static final class intake {
    public static final double firstIntakeTopSpeed = 0.3;
    public static final double firstIntakeBottomSpeed = 0.15;
    public static final double P = 1;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0;
    public static final double zI = 0;
    public static final double kMaxOutput = 0.8;
    public static final double kMinOutput = 0.1;
    public static final int motorChannel = 11;

    public static final int motorChannelTop = 9;
    public static final double kP = 1;
    public static final double kI = 0.000000;
    public static final double kD = 0.0;
    public static final double kIz = 0;
    public static final double kFF = 0;
  }

  public static final class flywheel {
    public static final int waitForShooterSecs = 10;
    public static final double feedMotorSpeed = 0.5;
    public static final double shooterMotorSpeed = 1600;
    public static final double kP = 0.0012;
    public static final double kI = 0.000000;
    public static final double kD = 0.04;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.7;
    public static final double kMinOutput = -0.7;
    public static final double speedTolerance = 50.0;
    public static final int motorChanelTop = 14;
    public static final int motorChanelBottom = 17;
    // public static final double halfspeed = shooterMotorSpeed/2;
  }

  public static final class arm {
    public static final double kP = 0.04;
    public static final double kI = 0.000000;
    public static final double kD = 5;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.7;
    public static final double kMinOutput = -0.7;
    public static final double speedTolerance = 50.0;
    public static final double maxOutput = .4;
    public static final double minOutput = -.4;
    public static final float forwardLimit = 15f;
    public static final float reverseLimit = -95f;
    public static final double tolerance = 2.0;
    public static final int motorChanel = 12;
    public static final double incrementValue = 2.5;

    public static final double feedforwardKs = .01; // units
    public static final double feedforwardKv = .01; // units * seconds / radians
    public static final double feedforwardKg = .01; // units

    public static final double positionIntake = -94;
    public static final double positionShoot = -52;
    public static final double positionUpright = 0;
  }

  public static final class TurnPID {
    public static final double P = 0.08;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double f = 0.0;
    public static final double Tolerance = 2.0; // Measured in degrees
    public static final double TurnRateTolerance = 20; // Degrees per second
  }

  public static final class climber {
    public static final float upLeftHeight = 99f;
    public static final float downLeftHeight = -5f;
    public static final float upRightHeight = -81f;
    public static final float downRightHeight = 5f;
    public static final int leftMotorChanel = 15;
    public static final int rightMotorChanel = 16;
    public static final double kP = 0.03;
    public static final double kI = 0.000000;
    public static final double kD = 0.0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double maxOutput = .4;
    public static final double minOutput = -.4;
    public static final double tolerance = 5.0;
  }

  public static final class climberLeveling {
    public static final double kP = 0.1;
    public static final double kI = 0.000000;
    public static final double kD = 0.0;
    public static final double positionTolerance = 2.0; // Measured in degrees
    public static final double velocityTolerance = 5; // Degrees per seconds
    public static final double targetValue = 0;
  }

  public static final class moveCommand {
    public static final double lP = 1.0;
    public static final double lI = 0.0;
    public static final double lD = 0.0;
    public static final double lf = 0.0;
    public static final double lMaxAlignSpeed = 0.5; // Meters per second
    public static final double lAlignTolerance = 0.1; // Meters
    public static final double rP = 1.0;
    public static final double rI = 0.0;
    public static final double rD = 0.0;
    public static final double rf = 0.0;
    public static final double rMaxAlignSpeed = 0.5;
    public static final double rAlignTolerance = 0.2;
    public static final double HighSpeed = 0.9;
    public static final double LowSpeed = 0.3;
  }

  public static final class vision {
    // Constants such as camera and target height stored. Change per robot and goal!
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(22);

    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(25);

    // How far from the target we want to be
    public static final double goalRangeMeters = Units.feetToMeters(10);

    public static final double linearP = 2;
    public static final double linearI = 0.0;
    public static final double linearD = 0.05;

    public static final double angularP = 1.84;
    public static final double angularI = 0.08;
    public static final double angularD = 0.1;

    public static final double speakerAimCloseRange = Units.feetToMeters(3.5);
    public static final double speakerAimCloseAngle = -54;
    public static final double speakerAimFarRange = Units.feetToMeters(9);
    public static final double speakerAimFarAngle = -37;
  }
}
