// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;

//import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

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
      new Translation2d(+KINEMATICS_CHASSIS_WIDTH / 2, +KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(+KINEMATICS_CHASSIS_WIDTH / 2, -KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(-KINEMATICS_CHASSIS_WIDTH / 2, +KINEMATICS_CHASSIS_LENGTH / 2),
      new Translation2d(-KINEMATICS_CHASSIS_WIDTH / 2, -KINEMATICS_CHASSIS_LENGTH / 2)
    );

    public static final double RADIUS_DRIVE_CHASSIS = Math.sqrt((KINEMATICS_CHASSIS_LENGTH/2)*(KINEMATICS_CHASSIS_LENGTH/2) + (KINEMATICS_CHASSIS_WIDTH/2)*(KINEMATICS_CHASSIS_WIDTH/2));
  }

  public static final class ModuleConstants {

    //Robot Geometry
    public static final double MODULE_WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double MODULE_DRIVE_GEAR_RATIO = 5.9 / 1.0; // Drive ratio of 8.14 : 1
    public static final double MODULE_TURN_GEAR_RATIO = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1
    public static final double MODULE_DRIVE_ROTATIONS_TO_METERS = ((MODULE_WHEEL_DIAMETER * Math.PI) / MODULE_DRIVE_GEAR_RATIO);//1.05; //1.0475;/// 0.77; -- multiplier to compensate for tread thickness
    public static final double MODULE_TURN_ROTATION_TO_RADIANS = MODULE_TURN_GEAR_RATIO * 2 * Math.PI;
    public static final double MODULE_DRIVE_RPM_TO_MPS = MODULE_DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double TurningEncoderRPM2RadPerSec = MODULE_TURN_ROTATION_TO_RADIANS / 60.0;
}

  public static class SwerveSubsystemConstants {


    // DRIVE Motor Ports
    public static final int ID_FRONT_LEFT_DRIVE  = 4;
    public static final int ID_BACK_LEFT_DRIVE   = 2;
    public static final int ID_FRONT_RIGHT_DRIVE = 6;
    public static final int ID_BACK_RIGHT_DRIVE  = 8;

    // TURNING Motor Ports
    public static final int ID_FRONT_LEFT_TURN  = 3;
    public static final int ID_BACK_LEFT_TURN   = 1;
    public static final int ID_FRONT_RIGHT_TURN = 5;
    public static final int ID_BACK_RIGHT_TURN  = 7;

    // CANCoder Ids
    public static final int ID_FRONT_LEFT_ENCODER_ABSOLUTE  = 6;
    public static final int ID_BACK_LEFT_ENCODER_ABSOLUTE   = 8;
    public static final int ID_FRONT_RIGHT_ENCODER_ABSOLUTE = 5;
    public static final int ID_BACK_RIGHT_ENCODER_ABSOLUTE  = 7;

    // Invert booleans | We use MK4i modules so the turning motors are inverted
    public static final boolean REVERSED_ENCODER_TURN     = true;
    public static final boolean REVERSED_ENCODER_DRIVE    = false;
    public static final boolean REVERSED_ENCODER_ABSOLUTE = false;
    public static final boolean REVERSED_GYRO             = false;

    // Invert Specific Motors
    public static final boolean REVERSED_FRONT_LEFT_MOTOR_DRIVE  = false;//true;//false;
    public static final boolean REVERSED_FRONT_RIGHT_MOTOR_DRIVE = true;//false;//true;
    public static final boolean REVERSED_BACK_LEFT_MOTOR_DRIVE   = false;//true;//false;
    public static final boolean REVERSED_BACK_RIGHT_MOTOR_DRIVE  = true;//false;//true;

    // Turning encoder offsets
    public static final double OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE  = Math.toRadians(-135.3515 + 180);
    public static final double OFFSET_BACK_LEFT_ENCODER_ABSOLUTE   = Math.toRadians(-153.8964 + 180);
    public static final double OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE = Math.toRadians(18.98430);
    public static final double OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE  = Math.toRadians(152.1386);

    // Robot drive speeds
    public static final double LIMIT_HARD_SPEED_DRIVE = Units.feetToMeters(17.385); // hard limit for speed of chassis
    public static final double LIMIT_SOFT_SPEED_DRIVE = Units.feetToMeters(17.385); // soft limit for speed of chassis

    // Robot turning speeds
    public static final double LIMIT_SOFT_SPEED_TURN =  2.5*Math.PI; // soft limit for module rotation
  }

  public static class IntakeSubsystemConstants {
    public static int ID_MOTOR_ROLLER       = 32;
    public static int ID_MOTOR_INTAKE_PIVOT = 31;

    public static double INTAKE_PIVOT_GEAR_RATIO = (1/50);
    public static double INTAKE_PIVOT_ROTATIONS_TO_DEGRESS = INTAKE_PIVOT_GEAR_RATIO * 2*Math.PI;
    public static double Forward_IntakePivot_Position = 0.0;    
    public static double Reverse_IntakePivot_Position = 18.0;

  }
  
  public static class ConveyerSubsystemConstants {
    public static int ID_MOTOR_CONVEYER_LEFT  = 11;
    public static int ID_MOTOR_CONVEYER_RIGHT = 12;
    public static int ID_SENSOR_LINEBREAK     = 2;
  }

  public static class ShooterSubsystemConstants {
    public static int ID_MOTOR_FEEDER          = 51;
    public static int ID_MOTOR_SHOOTER_LEFT    = 52;
    public static int ID_MOTOR_SHOOTER_RIGHT   = 53;
    public static int ID_MOTOR_TILTER          = 54;
    public static int ID_MOTOR_TILTER_FOLLOWER = 55;
    public static int ID_MOTOR_EXTENSION       = 56;
    public static int ID_SHOOTER_BEAMBREAK     = 0;
    
    public static double SHOOTER_EXTENSION_GEAR = 25/1;

    public static double EXTENSION_ROT_TO_HEIGHT = 5.5 / SHOOTER_EXTENSION_GEAR;

    public static double MOTOR_FEEDER_GEARRATIO = 1 / 3;
    public static double SHOOTER_GEAR_RATIO = 60/1; // 36 : 1
    public static double SHOOTER_TICKS_TO_DEGREES = 360 / SHOOTER_GEAR_RATIO;
    public static double SHOOTER_HEIGHT = 23.5;
    public static double SHOOTER_HOME_ANGLE = 60.0;
    public static final double SHOOTER_WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double SHOOTER_LAUNCH_GEAR_RATIO = 1.0 / 1.0; 
    public static final double SHOOTER_ROTATIONS_TO_METERS = ((SHOOTER_WHEEL_DIAMETER * Math.PI) / SHOOTER_LAUNCH_GEAR_RATIO);
    public static final double SHOOTER_RPM_TO_MPS = SHOOTER_ROTATIONS_TO_METERS / 60.0;
  }

  public static class ClimberSubsystemConstants {
    public static int ID_LEFT_WINCH  = 61; 
    public static int ID_RIGHT_WINCH = 62;

    public static double climberUpPosition = 50; //TODO: Measure this.


  }

  public static class VisionConstants {
    public static class defaultSTD {
        public static Vector<N3> singleTagStD = VecBuilder.fill(2, 2, 4);
    }
  }

  public static class BlingConstants {
    public static int ID_LED_STRIP   = 9;
    public static int ledStripLength = 92;
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
    public static class PathFindingConstraints{
      public static PathConstraints kConstraints = new PathConstraints(
              1.5,//3.25,
        1.5,//3.25,
        Math.toRadians(360),
        Math.toRadians(720));
    }
  }

}