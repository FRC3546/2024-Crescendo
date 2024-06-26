// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.geometry.Translation3d;
// import swervelib.math.Matter;
// import swervelib.parser.PIDFConfig;
// import edu.wpi.first.math.util.Units;
// import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  // public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  // public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Shooter{

    public static final int maxShooterRPM = 5450;
    public static final int speakerRPM = 3000;
    public static final int ampRPM = 1600;
    
  }

  public static final class Arm{

    public static final double armAngleConstant = 0.008333333;

    public static final double ampArmAngle = 0.573005 + armAngleConstant;
    public static final double speakerArmAngle = 0.3698222 + armAngleConstant;
    public static final double intakeArmAngle = 0.3142666 + armAngleConstant;
    public static final double stageShotArmAngle = (0.429595 - 0.0023) + armAngleConstant;
    public static final double startingConfigArmAngle = 0.5253777 + armAngleConstant;
    public static final double lowestArmAngle = 0.3142666 + armAngleConstant;
    public static final double highestArmAngle = 0.577172 + armAngleConstant;
  }

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }


  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0;
    public static final double LEFT_Y_DEADBAND  = 0;
    public static final double RIGHT_X_DEADBAND = 0;
    public static final double TURN_CONSTANT    = 6;
  }
}
