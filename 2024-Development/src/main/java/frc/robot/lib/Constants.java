// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class KinematicsConstants {
    public static final double KINEMATICS_CHASSIS_WIDTH = Units.inchesToMeters(22.5); // Distance between right and left wheels
    public static final double KINEMATICS_CHASSIS_LENGTH = Units.inchesToMeters(22.5); // Distance between front and back wheels
    public static final SwerveDriveKinematics KINEMATICS_DRIVE_CHASSIS = new SwerveDriveKinematics(
      new Translation2d(-KINEMATICS_CHASSIS_WIDTH / 2, +KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(+KINEMATICS_CHASSIS_WIDTH / 2, +KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(-KINEMATICS_CHASSIS_WIDTH / 2, -KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(KINEMATICS_CHASSIS_WIDTH / 2, -KINEMATICS_CHASSIS_LENGTH / 2)
    );

    public static final double RADIUS_DRIVE_CHASSIS = Math.sqrt(KINEMATICS_CHASSIS_LENGTH*KINEMATICS_CHASSIS_LENGTH + KINEMATICS_CHASSIS_WIDTH*KINEMATICS_CHASSIS_WIDTH);
  }

  public static final class ModuleConstants {

    //Robot Geometry
    public static final double MODULE_WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double MODULE_DRIVE_GEAR_RATIO = 8.14 / 1.0; // Drive ratio of 8.14 : 1
    public static final double MODULE_TURN_GEAR_RATIO = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1
    public static final double MODULE_DRIVE_ROTATIONS_TO_METERS = ((MODULE_WHEEL_DIAMETER * Math.PI) / MODULE_DRIVE_GEAR_RATIO);
    public static final double MODULE_TURN_ROTATIONS_TO_RADIANS = MODULE_TURN_GEAR_RATIO * 2 * Math.PI;
    public static final double MODULE_DRIVE_RPM_TO_MPS = MODULE_DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double TurningEncoderRPM2RadPerSec = MODULE_TURN_ROTATIONS_TO_RADIANS / 60.0;
}

  public static class SwerveSubsystemConstants {


    // DRIVE Motor Ports
    public static final int ID_FRONT_LEFT_DRIVE = 4;
    public static final int ID_BACK_LEFT_DRIVE = 2;
    public static final int ID_FRONT_RIGHT_DRIVE = 6;
    public static final int ID_BACK_RIGHT_DRIVE = 8;

    // TURNING Motor Ports
    public static final int ID_FRONT_LEFT_TURN = 3;
    public static final int ID_BACK_LEFT_TURN = 1;
    public static final int ID_FRONT_RIGHT_TURN = 5;
    public static final int ID_BACK_RIGHT_TURN = 7;

    // CANCoder Ids
    public static final int ID_FRONT_LEFT_ENCODER_ABSOLUTE = 6;
    public static final int ID_BACK_LEFT_ENCODER_ABSOLUTE = 8;
    public static final int ID_FRONT_RIGHT_ENCODER_ABSOLUTE = 5;
    public static final int ID_BACK_RIGHT_ENCODER_ABSOLUTE = 7;

    // Invert booleans | We use MK4i modules so the turning motors are inverted
    public static final boolean REVERSED_ENCODER_TURN = true;
    public static final boolean REVERSED_ENCODER_DRIVE = false;
    public static final boolean REVERSED_ENCODER_ABSOLUTE = false;
    public static final boolean REVERSED_GYRO = true;

    // Invert Specific Motors

    public static final boolean REVERSED_FRONT_LEFT_MOTOR_DRIVE = false;
    public static final boolean REVERSED_FRONT_RIGHT_MOTOR_DRIVE = true;
    public static final boolean REVERSED_BACK_LEFT_MOTOR_DRIVE = false;
    public static final boolean REVERSED_BACK_RIGHT_MOTOR_DRIVE = true;

    // Turning encoder offsets
    public static final double OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE = Math.toRadians(-96.0 +  7.6);
    public static final double OFFSET_BACK_LEFT_ENCODER_ABSOLUTE  = Math.toRadians(40 + 2.78);
    public static final double OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE= Math.toRadians(-77.0 +  6);
    public static final double OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE = Math.toRadians( 62.0 + -3.6);

    // Robot drive speeds
    public static final double LIMIT_HARD_SPEED_DRIVE = 3.6; // hard limit for speed of chassis
    public static final double LIMIT_SOFT_SPEED_DRIVE = 1.0; // soft limit for speed of chassis

    // Robot turning speeds
    public static final double LIMIT_SOFT_SPEED_TURN = 1 * 2*Math.PI; // soft limit for module rotation

    // Robot acceleration
    public static final double LIMIT_SOFT_ACCELERATION_SPEED = 1; // soft limit for acceleration (M/S^2)
    public static final double LIMIT_SOFT_ACCELERATION_TURN = 1;  // soft limit for acceleration (M/S^2)
  }

  public static class IntakeSubsystemConstants {
    public static int ID_MOTOR_ROLLER = 9;
    public static int ID_MOTOR_INTAKE_PIVOT = 31;


    public static double INTAKE_PIVOT_GEAR_RATIO = (1/50);
    public static double INTAKE_PIVOT_ROTATIONS_TO_DEGRESS = INTAKE_PIVOT_GEAR_RATIO * 2*Math.PI;

    public static double Forward_IntakePivot_Position = -18.5;
    public static double Reverse_IntakePivot_Position = 0.0;

  }
  
  public static class ConveyerSubsystemConstants {
    public static int ID_MOTOR_CONVEYER_LEFT = 11;
    public static int ID_MOTOR_CONVEYER_RIGHT = 12;


    public static int ID_SENSOR_LINEBREAK = 1;
  }

  public static class ShooterSubsystemConstants {

    public static int ID_MOTOR_TILTER = 51;
    public static int ID_MOTOR_SHOOTER_LEADER = 52;
    public static int ID_MOTOR_SHOOTER_FOLLOWER = 53;


    public static int ID_MOTOR_FEEDER = 54;
    public static double MOTOR_FEEDER_GEARRATIO = 1 / 3;

    public static class ShooterBlockPneumatics {
      public static int CHANNEL_FORWARD = 1;
      public static int CHANNEL_REVERSE = 2;
    }

    public static int ID_MOTOR_CONVEYER = 50;
    public static int ID_SENSOR_LINEBREAK = 0;
  }

  public static final class OIConstants {
    public static final int ID_CONTROLLER_DRIVER = 0;
    public static final double CONTROLLER_DRIVER_DEADBAND = 0.05;

    // Joysticks
    public static final int CONTROLLER_DRIVER_Y = 1;
    public static final int CONTROLLER_DRIVER_X = 0;
    public static final int CONTROLLER_DRIVER_Z = 4;

    public static final int ID_CONTROLLER_OPERATOR = 1;
  }

  public static class AutonomousConstants{
    public static double LIMIT_AUTOSPEED_DRIVE = 0.30;
    public static double LIMIT_AUTOSPEED_ROTATE = 180;
  }
}