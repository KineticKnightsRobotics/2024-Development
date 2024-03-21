// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.lib.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;

//import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
  private final Intake SUBSYSTEM_INTAKE = new Intake();
  private final Conveyer SUBSYSTEM_CONVEYER = new Conveyer();
  private final Shooter SUBSYSTEM_SHOOTER = new Shooter();
  private final Climber SUBSYSTEM_CLIMBER = new Climber();
  private final SwerveDrive SUBSYSTEM_SWERVEDRIVE = new SwerveDrive();

  //private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


  //private Command lockCommand = new RunCommand(() -> SUBSYSTEM_SWERVEDRIVE.lockChassis(),SUBSYSTEM_SWERVEDRIVE);


  private final static CommandJoystick JOYSTICK_DRIVER = new CommandJoystick(OIConstants.ID_CONTROLLER_DRIVER);
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



  //private final SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SUBSYSTEM_SWERVEDRIVE.setDefaultCommand(
      new joystickDrive(
        SUBSYSTEM_SWERVEDRIVE, 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
        () -> true, 
        ()-> getAllianceFlip(),
        () -> 0.02
      )
    );


    configureNamedCommands();


    //autoChooser = AutoBuilder.buildAutoChooser();

    //SmartDashboard.putData("Autos", autoChooser);


    configureBindings();
  }

  private void configureBindings() {
    //TELEOP ROBOT TRIGGERED EVENTS _______________________________________________________________________________________________________________________________________________________________________

    //When Shooter falls under the home position, set it back to 0
    alwaysOn.whileTrue(SUBSYSTEM_SHOOTER.setExtensionSpeed(-0.02));
    ShooterUnderHome.onTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 2.0)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    //When Note is in the shooter, idle the flywheels and stop the conveyer.
    NoteInFeederTrigger.and(OP_1.negate()).whileTrue(SUBSYSTEM_SHOOTER.IdleShooter(1500,1500));
    NoteInFeederTrigger.negate().and(OP_1.negate()).whileTrue(SUBSYSTEM_SHOOTER.stopShooter());
    NoteInFeederTrigger.whileTrue(SUBSYSTEM_CONVEYER.setConveyerSpeed(0.0).alongWith(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0)));

    //TELEOP CONTROLS _____________________________________________________________________________________________________________________________________________________________________________________
    
    //Lock
    DRIVER_R2.whileTrue(
      new ParallelCommandGroup(
        new rotationTargetLockDrive(
            SUBSYSTEM_SWERVEDRIVE,   
            () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
            () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
            () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
            () -> true, 
            () -> getAllianceFlip(),
            () -> 0.02
          ),
          //SUBSYSTEM_SHOOTER.aimTilter(() -> SUBSYSTEM_SHOOTER.shooterInterpolator.interpolateAngle(SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())),
          SUBSYSTEM_SHOOTER.IdleShooter(4022,2681)
        )
    ).onFalse(SUBSYSTEM_SHOOTER.IdleShooter(1000, 1000));

    DRIVER_R1.and(ShooterAtAmp.negate()).whileTrue(
        SUBSYSTEM_SHOOTER.shoot(4022,2681, false)
    );
    DRIVER_R1.and(ShooterAtAmp).whileTrue(
        SUBSYSTEM_SHOOTER.spitOutNote()
    );



    /*
    DRIVER_R1.whileTrue(
      (SUBSYSTEM_SHOOTER.ampPostion() ?
        SUBSYSTEM_SHOOTER.spitOutNote()//SUBSYSTEM_SHOOTER.setFeederSpeed(0.8)//SUBSYSTEM_SHOOTER.shoot(500, 500, false)
        :           
        SUBSYSTEM_SHOOTER.shoot(4022,2681, false)
        )
    );
    */




    DRIVER_L1.and(NoteInConveyerTrigger.negate()).and(NoteInFeederTrigger.negate()).whileTrue(
      new SequentialCommandGroup(
        SUBSYSTEM_INTAKE.intakeDown(),
        SUBSYSTEM_CONVEYER.setConveyerSpeed(0.8),
        SUBSYSTEM_SHOOTER.loadGamePiece()
        //SUBSYSTEM_INTAKE.intakeUp()
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
      //new ParallelCommandGroup(
      SUBSYSTEM_SHOOTER.setExtensionHeight(0.0).andThen(
      SUBSYSTEM_SHOOTER.setTilter(() -> 5.0)
      ).until(() -> (Math.abs(SUBSYSTEM_SHOOTER.getTilterPosition()-5.0) < 0.05 && SUBSYSTEM_SHOOTER.getExtensionPosition() < 0.15)) // This is the dumbest fix of all time
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      //)
    );
    DRIVER_X.whileTrue(AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.84,7.73),new Rotation2d(-90.0)) , AutonomousConstants.PathFindingConstraints.kConstraints));
    DRIVER_B.whileTrue(AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.45,5.52),new Rotation2d(0.0)), AutonomousConstants.PathFindingConstraints.kConstraints));

    DRIVER_START.whileTrue(SUBSYSTEM_SWERVEDRIVE.zeroRobotHeading());

    //DRIVER_Y.whileTrue(SUBSYSTEM_SHOOTER.setShooterTEMP()).onFalse(SUBSYSTEM_SHOOTER.stopShooter());


    //OPERATOR CONTROLS________________________________________________________________________________________________________________________________________________________________

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


    OP_2.whileTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 10));
    OP_3.whileTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 20));
    OP_6.whileTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 25));
    OP_7.whileTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 30));
    OP_8.whileTrue(SUBSYSTEM_SHOOTER.setTilter(() -> 32));


    /*
    OP_2.whileTrue(SUBSYSTEM_SHOOTER.setFeederSpeed(-0.2)).onFalse(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0));
    OP_3.whileTrue(SUBSYSTEM_CONVEYER.setConveyerSpeed(-0.2)).onFalse(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0));
    OP_6.whileTrue(Commands.runOnce(() -> SUBSYSTEM_INTAKE.setRollerSpeed(-0.2))).onFalse(Commands.runOnce(() -> SUBSYSTEM_INTAKE.setRollerSpeed(0.0)));
    OP_7.whileTrue(SUBSYSTEM_SHOOTER.reverseShooter()).onFalse(SUBSYSTEM_SHOOTER.stopShooter());
    OP_8.onTrue(SUBSYSTEM_SHOOTER.stopShooter());
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
            //SUBSYSTEM_SHOOTER.setTilter(() -> SUBSYSTEM_SHOOTER.getTilterAimAngle(SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())),
         // SUBSYSTEM_SHOOTER.setTilter(() -> SUBSYSTEM_SHOOTER.shooterInterpolator.interpolateAngle(SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())),
           // SUBSYSTEM_SHOOTER.setTilter(SUBSYSTEM_SHOOTER.getTilterAimAngle(SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())),
          new rotationTargetLockDrive(SUBSYSTEM_SWERVEDRIVE,   
            () -> 0.0,//-JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
            () -> 0.0,//-JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
            () -> 0.0,//-JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
            () -> true, 
            () -> getAllianceFlip(),
            () -> 0.02
          )
        ),
        SUBSYSTEM_SHOOTER.shoot(4022,2681, false),
        SUBSYSTEM_SHOOTER.setTilter(() -> 0.0)
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

  /**
   * @return Whether or not we are on red alliance
   */
  public boolean getAllianceFlip() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

}
