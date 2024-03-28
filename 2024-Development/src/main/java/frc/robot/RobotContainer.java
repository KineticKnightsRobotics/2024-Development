// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.lib.Waypoint;
import frc.robot.lib.Constants.*;

//import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  //Init auto chooser
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Intake SUBSYSTEM_INTAKE = new Intake();
  public final Conveyer SUBSYSTEM_CONVEYER = new Conveyer();
  public final Shooter SUBSYSTEM_SHOOTER = new Shooter();
  public final Climber SUBSYSTEM_CLIMBER = new Climber();
  public final SwerveDrive SUBSYSTEM_SWERVEDRIVE = new SwerveDrive();
  public final Bling SUBSYSTEM_BLING = new Bling();

  //private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


  //private Command lockCommand = new RunCommand(() -> SUBSYSTEM_SWERVEDRIVE.lockChassis(),SUBSYSTEM_SWERVEDRIVE);


  public final static CommandJoystick JOYSTICK_DRIVER = new CommandJoystick(OIConstants.ID_CONTROLLER_DRIVER);
  Trigger DRIVER_A = new Trigger(JOYSTICK_DRIVER.button(1));
  Trigger DRIVER_B = new Trigger(JOYSTICK_DRIVER.button(2));
  Trigger DRIVER_X = new Trigger(JOYSTICK_DRIVER.button(3));
  Trigger DRIVER_Y = new Trigger(JOYSTICK_DRIVER.button(4));
  Trigger DRIVER_L1= new Trigger(JOYSTICK_DRIVER.button(5));
  Trigger DRIVER_R1= new Trigger(JOYSTICK_DRIVER.button(6));
  //Driver L2 right now is hardcoded to be percision mode!
  Trigger DRIVER_R2 = new Trigger(()-> JOYSTICK_DRIVER.getRawAxis(3)>=0.3);
  Trigger DRIVER_START= new Trigger(JOYSTICK_DRIVER.button(8));
  Trigger DRIVER_BACK = new Trigger(JOYSTICK_DRIVER.button(7));
  Trigger DRIVER_L3 = new Trigger(JOYSTICK_DRIVER.button(9));
  Trigger DRIVER_R3 = new Trigger(JOYSTICK_DRIVER.button(10));

  private final static CommandJoystick JOYSTICK_OPERATOR = new CommandJoystick(OIConstants.ID_CONTROLLER_OPERATOR);
  Trigger OP_1 = new Trigger(JOYSTICK_OPERATOR.button(1 ));
  Trigger OP_2 = new Trigger(JOYSTICK_OPERATOR.button(2 ));
  Trigger OP_3 = new Trigger(JOYSTICK_OPERATOR.button(3 ));
  Trigger OP_4 = new Trigger(JOYSTICK_OPERATOR.button(4 ));
  Trigger OP_5 = new Trigger(JOYSTICK_OPERATOR.button(5 ));
  Trigger OP_6 = new Trigger(JOYSTICK_OPERATOR.button(6 ));
  Trigger OP_7 = new Trigger(JOYSTICK_OPERATOR.button(7 ));
  Trigger OP_8 = new Trigger(JOYSTICK_OPERATOR.button(8 ));
  Trigger OP_9 = new Trigger(JOYSTICK_OPERATOR.button(9 ));
  Trigger OP_10= new Trigger(JOYSTICK_OPERATOR.button(10));
  Trigger OP_11= new Trigger(JOYSTICK_OPERATOR.button(11));
  Trigger OP_12= new Trigger(JOYSTICK_OPERATOR.button(12));
  Trigger OP_13= new Trigger(JOYSTICK_OPERATOR.button(13));
  Trigger OP_14= new Trigger(JOYSTICK_OPERATOR.button(14));
  Trigger OP_15= new Trigger(JOYSTICK_OPERATOR.button(15));
  Trigger OP_16= new Trigger(JOYSTICK_OPERATOR.button(16));
  Trigger OP_17 = new Trigger(JOYSTICK_OPERATOR.button(17));
  Trigger OP_18 = new Trigger(JOYSTICK_OPERATOR.button(18));
  Trigger OP_19 = new Trigger(JOYSTICK_OPERATOR.button(19));
  Trigger OP_20 = new Trigger(JOYSTICK_OPERATOR.button(20));
  Trigger OP_21 = new Trigger(JOYSTICK_OPERATOR.button(21));

  //Translation2d tag = fieldLayout.getTagPose(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 3).get().getTranslation().toTranslation2d();

  int ShooterRPM = 4022;

  //ROBOT TRIGGERS
  Trigger alwaysOn = new Trigger(() -> true);
    
  Trigger ShooterAtAmp = new Trigger(() -> SUBSYSTEM_SHOOTER.getExtensionPosition() > 2);

  Trigger ShooterUnderHome = new Trigger(() -> SUBSYSTEM_SHOOTER.getTilterPosition() < -3.0);

  Trigger ShooterAtHomeTrigger = new Trigger(() -> SUBSYSTEM_SHOOTER.getTilterPosition() <= 10.0 && SUBSYSTEM_SHOOTER.getTilterPosition() > -3.0 && SUBSYSTEM_SHOOTER.getExtensionPosition() < 0.10);

  Trigger NoteInConveyerTrigger = new Trigger(() -> SUBSYSTEM_CONVEYER.getLineBreak());

  Trigger NoteInFeederTrigger = new Trigger(() -> SUBSYSTEM_SHOOTER.getLineBreak());//SUBSYSTEM_SHOOTER::getLineBreak);

  Trigger DriveCurrentLimitTrigger = new Trigger(()->SUBSYSTEM_SWERVEDRIVE.getCurrentDrive()>150);

  Trigger LockDriveTrigger = new Trigger(()-> SUBSYSTEM_SWERVEDRIVE.getLockTimer() >=0.521);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SUBSYSTEM_SWERVEDRIVE.setDefaultCommand(
      new joystickDrive(
        SUBSYSTEM_SWERVEDRIVE, 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
        () -> true, 
        () -> 0.02
      )
    );
    
    configureNamedCommands();
    configureBindings();
  }

  private void configureBindings() {
    //TELEOP ROBOT TRIGGERED EVENTS _______________________________________________________________________________________________________________________________________________________________________

    alwaysOn.whileTrue(SUBSYSTEM_SHOOTER.setExtensionSpeed(-0.02));
    NoteInFeederTrigger.whileTrue(SUBSYSTEM_BLING.blinkNote());
    NoteInFeederTrigger.and(OP_1.negate()).whileTrue(SUBSYSTEM_SHOOTER.IdleShooter(1500,1500));
    NoteInFeederTrigger.negate().and(OP_1.negate()).whileTrue(SUBSYSTEM_SHOOTER.stopShooter());
    NoteInFeederTrigger.whileTrue(SUBSYSTEM_CONVEYER.setConveyerSpeed(0.0).alongWith(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0)));

    //TELEOP CONTROLS _____________________________________________________________________________________________________________________________________________________________________________________
  
    //Aim at speaker
    DRIVER_R2.whileTrue(
      new ParallelCommandGroup(
        new rotationTargetLockDrive(
            SUBSYSTEM_SWERVEDRIVE,   
            () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
            () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
            () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
            () -> true, 
            () -> 0.02
          ),
          SUBSYSTEM_SHOOTER.autoTilter(() -> SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())
        )
    ).onFalse(SUBSYSTEM_SHOOTER.IdleShooter(1000, 1000).andThen(SUBSYSTEM_SHOOTER.setTilter(()->0.0)));

    DRIVER_R1.and(ShooterAtAmp.negate()).whileTrue(
        SUBSYSTEM_SHOOTER.shoot(4500,3700, false)
    );
    DRIVER_R1.and(ShooterAtAmp).whileTrue(
        SUBSYSTEM_SHOOTER.spitOutNote()
    );

    DRIVER_L1.and(NoteInConveyerTrigger.negate()).and(NoteInFeederTrigger.negate()).whileTrue(
      new SequentialCommandGroup(
        SUBSYSTEM_INTAKE.intakeDown(),
        SUBSYSTEM_CONVEYER.setConveyerSpeed(0.8),
        SUBSYSTEM_SHOOTER.loadGamePiece()
      ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    )
    .onFalse(SUBSYSTEM_INTAKE.intakeUp());

    DRIVER_A.whileTrue(
      new ParallelCommandGroup(
        SUBSYSTEM_SHOOTER.setExtensionHeight(6),
        SUBSYSTEM_SHOOTER.setTilter(() -> 155)
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    )
    .whileFalse(
      SUBSYSTEM_SHOOTER.setExtensionHeight(0.0).andThen(
      SUBSYSTEM_SHOOTER.setTilter(() -> 0.0)
      ).until(() -> (Math.abs(SUBSYSTEM_SHOOTER.getTilterPosition()-5.0) < 0.05 && SUBSYSTEM_SHOOTER.getExtensionPosition() < 0.15)) // This is the dumbest fix of all time
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
    
    DRIVER_X.whileTrue(SwerveDrive.pathFind(Waypoint.Amp.blue,Waypoint.Amp.red));

    DRIVER_START.whileTrue(SUBSYSTEM_SWERVEDRIVE.zeroRobotHeading());

    //OPERATOR CONTROLS________________________________________________________________________________________________________________________________________________________________


    OP_1.whileTrue(SUBSYSTEM_SHOOTER.setTilter(()->12.5)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_2.whileTrue(SUBSYSTEM_SHOOTER.setTilter(()->17.5)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_3.whileTrue(SUBSYSTEM_SHOOTER.setTilter(()->22.5)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_6.whileTrue(SUBSYSTEM_SHOOTER.setTilter(()->27.5)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_7.whileTrue(SUBSYSTEM_SHOOTER.setTilter(()->32.5)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    //OP_8.whileTrue(SUBSYSTEM_SHOOTER.setTilter(()->32));
    OP_8.whileTrue(SUBSYSTEM_SHOOTER.setTilter(()-> SmartDashboard.getNumber("! Shooter Angle",0.0)));



    /*
    //Reverse override subsystems
    OP_1.whileTrue(
      new ParallelCommandGroup(
        SUBSYSTEM_SHOOTER.setFeederSpeed(-0.2),
        SUBSYSTEM_CONVEYER.setConveyerSpeed(-0.2),
        Commands.run(() -> SUBSYSTEM_INTAKE.setRollerSpeed(0.2)),
        SUBSYSTEM_SHOOTER.reverseShooter()
        
      )
    ).onFalse(
      new ParallelCommandGroup(
        SUBSYSTEM_SHOOTER.setFeederSpeed(0.0),
        SUBSYSTEM_CONVEYER.setConveyerSpeed(0.0),
        Commands.runOnce(() -> SUBSYSTEM_INTAKE.setRollerSpeed(0.0)),
        SUBSYSTEM_SHOOTER.stopShooter()
      )
    );

    //OP_14.whileTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 25.0));

    OP_2.whileTrue(SUBSYSTEM_SHOOTER.setFeederSpeed(-0.2)).onFalse(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0));
    OP_3.whileTrue(SUBSYSTEM_CONVEYER.setConveyerSpeed(-0.2)).onFalse(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0));
    OP_6.whileTrue(Commands.runOnce(() -> SUBSYSTEM_INTAKE.setRollerSpeed(-0.2))).onFalse(Commands.runOnce(() -> SUBSYSTEM_INTAKE.setRollerSpeed(0.0)));
    OP_7.whileTrue(SUBSYSTEM_SHOOTER.reverseShooter()).onFalse(SUBSYSTEM_SHOOTER.stopShooter());
    OP_8.onTrue(SUBSYSTEM_SHOOTER.stopShooter().alongWith(SUBSYSTEM_CONVEYER.setConveyerSpeed(0.0)));
    */
    //Climber controls
    OP_4.whileTrue(SUBSYSTEM_CLIMBER.setWinchSpeed(0.9)).onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0.0));
    OP_9.whileTrue(SUBSYSTEM_CLIMBER.setWinchSpeed(-0.9)).onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0));
    //Manual Shooter Extension
    OP_5.whileTrue(SUBSYSTEM_SHOOTER.setExtensionSpeed(0.5)).onFalse(SUBSYSTEM_SHOOTER.setExtensionSpeed(0.0));
    OP_10.whileTrue(SUBSYSTEM_SHOOTER.setExtensionSpeed(-0.5)).onFalse(SUBSYSTEM_SHOOTER.setExtensionSpeed(0.0));
    //Manual Tilter Angles
    OP_12.whileTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 0)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    //Override bring the shooter down
    OP_16.whileTrue(SUBSYSTEM_SHOOTER.setTilterVoltage(-2)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    //Override Shooter Buttons.
    OP_19.whileTrue(SUBSYSTEM_SHOOTER.shoot(4000, 4000, true)).onFalse(SUBSYSTEM_SHOOTER.stopShooter());
    OP_20.whileTrue(SUBSYSTEM_SHOOTER.setFeederSpeed(0.8)).onFalse(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0));
    //Override zero tilter THIS BREAKS THE CODE IF YOU ZERO WHILE AT A NON 0 ANGLE.
    OP_17.and(OP_18).onTrue(SUBSYSTEM_SHOOTER.zeroTilter(0.0));
    //Override start shooter idle.
    OP_21.onTrue(SUBSYSTEM_SHOOTER.IdleShooter(1000, 1000));

    //SYS ID CONTROLS _________________________________________________________________________________________________________________________________________________________________
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("ShootNoAim", SUBSYSTEM_SHOOTER.shoot(4022, 2681, false).andThen(SUBSYSTEM_SHOOTER.IdleShooter(1000, 1000)));

    NamedCommands.registerCommand("Shoot",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new rotationTargetLockDrive(SUBSYSTEM_SWERVEDRIVE,   
            () -> 0.0,
            () -> 0.0,
            () -> 0.0,
            () -> true,
            () -> 0.02
          ),
          SUBSYSTEM_SHOOTER.autoTilter(() -> SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())
        ),
        SUBSYSTEM_SHOOTER.shoot(4022,2681, false),
        SUBSYSTEM_SHOOTER.setTilter(() -> 0.0).until(() -> SUBSYSTEM_SHOOTER.getTilterPosition() < 0.05)
      )
    );

    NamedCommands.registerCommand("IntakeDown", SUBSYSTEM_INTAKE.intakeDown());

    NamedCommands.registerCommand("CaptureNote", 
    new SequentialCommandGroup(
        SUBSYSTEM_INTAKE.intakeDown(),
        SUBSYSTEM_CONVEYER.setConveyerSpeed(0.2),
        SUBSYSTEM_SHOOTER.loadGamePiece(),
        SUBSYSTEM_SHOOTER.setFeederSpeed(0.0),
        SUBSYSTEM_CONVEYER.setConveyerSpeed(0.0),
        SUBSYSTEM_INTAKE.intakeUp()
        )
    );

    NamedCommands.registerCommand("ZeroRobotHeading", SUBSYSTEM_SWERVEDRIVE.zeroRobotHeading().andThen(SUBSYSTEM_SHOOTER.setExtensionSpeed(-0.02)));

    NamedCommands.registerCommand(
      "ZeroTilter",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(0.50), 
          SUBSYSTEM_SHOOTER.setTilter(() -> 20)),
        new WaitCommand(0.75),
        SUBSYSTEM_SHOOTER.zeroTilter(0.0)
      )
    );


    /*
    NamedCommands.registerCommand("ResetModulePosition", SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());
    NamedCommands.registerCommand("IntakeDown" , SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Forward_IntakePivot_Position));
    NamedCommands.registerCommand("IntakeUp" , SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Reverse_IntakePivot_Position));
    NamedCommands.registerCommand("AutoConveyer", new intakeLineBreak(SUBSYSTEM_CONVEYER,SUBSYSTEM_INTAKE));
    NamedCommands.registerCommand("AutoAimSpeaker", new autoAimSpeaker(SUBSYSTEM_SHOOTER));
    NamedCommands.registerCommand("AutoRunShooter", new autoRunShooter(SUBSYSTEM_SHOOTER/*,SUBSYSTEM_CONVEYER,2500.0));
    NamedCommands.registerCommand("AutoSetShooterIdle", new autoSetShooterIdle(SUBSYSTEM_SHOOTER));
    NamedCommands.registerCommand("AutoLoadShooter", new loadShooterAuto(SUBSYSTEM_CONVEYER,SUBSYSTEM_SHOOTER));
    NamedCommands.registerCommand("ShooterDown", SUBSYSTEM_SHOOTER.setTilter(0.0));
    
    
   */
  }


  public Command getAutonomousCommand() {
        //return new PathPlannerAuto("TwoNoteAuto");
        //return SUBSYSTEM_SHOOTER.setFeederSpeed(0.5);


        return new PathPlannerAuto("FourNotePP");

        //return new PathPlannerAuto("US4NoteAuto");
  } 

  public static boolean DRIVER_LT() {
    return JOYSTICK_DRIVER.getRawAxis(2) > 0.5;
  }
 /*public static Trigger rightTrigger(){
  return new Trigger(()-> JOYSTICK_DRIVER.getRawAxis(3)>=0.3);
}*/
 public static double DRIVER_RT() {
    return JOYSTICK_DRIVER.getRawAxis(2);
  }
}
