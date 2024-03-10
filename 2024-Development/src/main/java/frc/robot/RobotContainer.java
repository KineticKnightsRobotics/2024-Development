// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.lib.Constants;
import frc.robot.lib.Constants.OIConstants;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake SUBSYSTEM_INTAKE = new Intake();
  private final Conveyer SUBSYSTEM_CONVEYER = new Conveyer();
  private final Shooter SUBSYSTEM_SHOOTER = new Shooter();
  private final Climber SUBSYSTEM_CLIMBER = new Climber();
  private final SwerveDrive SUBSYSTEM_SWERVEDRIVE = new SwerveDrive();

  private final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


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

private final static CommandJoystick JOYSTICK_SYSID = new CommandJoystick(2);
  Trigger SYSID_1 = new Trigger(JOYSTICK_SYSID.button(1 ));
  Trigger SYSID_2 = new Trigger(JOYSTICK_SYSID.button(2 ));
  Trigger SYSID_3 = new Trigger(JOYSTICK_SYSID.button(3 ));
  Trigger SYSID_4 = new Trigger(JOYSTICK_SYSID.button(4 ));
  Trigger SYSID_5 = new Trigger(JOYSTICK_SYSID.button(5 ));
  Trigger SYSID_6 = new Trigger(JOYSTICK_SYSID.button(6 ));
  Trigger SYSID_7 = new Trigger(JOYSTICK_SYSID.button(7 ));
  Trigger SYSID_8 = new Trigger(JOYSTICK_SYSID.button(8 ));
  Trigger SYSID_9 = new Trigger(JOYSTICK_SYSID.button(9 ));
  Trigger SYSID_10= new Trigger(JOYSTICK_SYSID.button(10));
  Trigger SYSID_11= new Trigger(JOYSTICK_SYSID.button(11));
  Trigger SYSID_12= new Trigger(JOYSTICK_SYSID.button(12));
  Trigger SYSID_13= new Trigger(JOYSTICK_SYSID.button(13));
  Trigger SYSID_14= new Trigger(JOYSTICK_SYSID.button(14));
  Trigger SYSID_15= new Trigger(JOYSTICK_SYSID.button(15));
  Trigger SYSID_16= new Trigger(JOYSTICK_SYSID.button(16));
  Trigger SYSID_17 = new Trigger(JOYSTICK_SYSID.button(17));
  Trigger SYSID_18 = new Trigger(JOYSTICK_SYSID.button(18));
  Trigger SYSID_19 = new Trigger(JOYSTICK_SYSID.button(19));
  Trigger SYSID_20 = new Trigger(JOYSTICK_SYSID.button(20));
  Trigger SYSID_21 = new Trigger(JOYSTICK_SYSID.button(21));
  Trigger SYSID_22 = new Trigger(JOYSTICK_SYSID.button(22));
  Trigger SYSID_23 = new Trigger(JOYSTICK_SYSID.button(23));
  Trigger SYSID_24 = new Trigger(JOYSTICK_SYSID.button(24));
boolean toggle =false;


  //Translation2d tag = fieldLayout.getTagPose(DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 3).get().getTranslation().toTranslation2d();

  int ShooterRPM = 4022;

  //ROBOT TRIGGERS

  Trigger ShooterAtHomeTrigger = new Trigger(() -> SUBSYSTEM_SHOOTER.getTilterPosition() <= 10.0);
  Trigger NoteInConveyerTrigger = new Trigger(() -> SUBSYSTEM_CONVEYER.getLineBreak());
  Trigger NoteInFeederTrigger = new Trigger(() -> SUBSYSTEM_SHOOTER.getLineBreak());//SUBSYSTEM_SHOOTER::getLineBreak);
  Trigger DriveCurrentLimitTrigger = new Trigger(()->SUBSYSTEM_SWERVEDRIVE.getCurrentDrive()>150);
  Trigger LockDriveTrigger = new Trigger(()-> SUBSYSTEM_SWERVEDRIVE.getLockTimer() >=0.521);
  Trigger toggleTrigger = new Trigger(()-> SUBSYSTEM_CONVEYER.getToggle());
 // Trigger sensorTrigger = new Trigger(() -> SUBSYSeo TEM_CONVEYER.getLineBreak());

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


    NamedCommands.registerCommand("IntakeDown", SUBSYSTEM_INTAKE.intakeDown());

    NamedCommands.registerCommand("TempShoot", SUBSYSTEM_SHOOTER.shoot(4022, 2681, false));

    NamedCommands.registerCommand("Shoot",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          SUBSYSTEM_SHOOTER.aimTilter(() -> SUBSYSTEM_SHOOTER.shooterInterpolator.interpolateAngle(SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())),
          new rotationTargetLockDrive(SUBSYSTEM_SWERVEDRIVE,   
            () -> 0.0,//-JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
            () -> 0.0,//-JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
            () -> 0.0,//-JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
            () -> true, 
            () -> 0.02
          )
        ),
        SUBSYSTEM_SHOOTER.shoot(4022,2681, false)
      )
    );

    NamedCommands.registerCommand("CapturePiece", new SequentialCommandGroup(SUBSYSTEM_INTAKE.intakeDown(),SUBSYSTEM_CONVEYER.intakeGamePiece(),SUBSYSTEM_INTAKE.intakeUp()));


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
    
    //intake down, feed note to panel, set intake up)
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
    /*
    DRIVER_B.onTrue(SUBSYSTEM_SWERVEDRIVE.zeroRobotHeading());
    DRIVER_R1.whileTrue(
      new SequentialCommandGroup(
        SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Forward_IntakePivot_Position),
        new intakeLineBreak(SUBSYSTEM_CONVEYER,SUBSYSTEM_INTAKE),
        SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Reverse_IntakePivot_Position)
      )
    );
    DRIVER_R1.onFalse(SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Reverse_IntakePivot_Position));
    DRIVER_L1.whileTrue(
      new ParallelCommandGroup(  
        //new SHOOTER_runFeeder(-0.40, SUBSYSTEM_SHOOTER),
        new setRollerSpeed(SUBSYSTEM_INTAKE, -0.8),
        new runConveyer(0.4, SUBSYSTEM_CONVEYER),
        new setShooterSpeed(-0.30, SUBSYSTEM_SHOOTER)
      )
    );
   

      /*rightTrigger().whileTrue( new PIDCommand(
      new PIDController(0.05,0,0),
      // Close the loop on the turn rate
      SUBSYSTEM_SWERVEDRIVE::getRobotHeading,
      // Setpoint is 0
      0,
      // Pipe the output to the turning controls
      output ->  new joystickDrive(
        SUBSYSTEM_SWERVEDRIVE, 
        () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
        () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
        () -> output, 
        () -> true, 
        () -> 0.02
      ),
      // Require the robot drive
      SUBSYSTEM_SWERVEDRIVE));
    */


    //TELEOP CONTROLS _________________________________________________________________________________________________________________________________________________________________
    DRIVER_R2.whileTrue(
    
    new ParallelCommandGroup(
      new rotationTargetLockDrive(SUBSYSTEM_SWERVEDRIVE,   
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
        () -> true, 
        () -> 0.02
      ),
      SUBSYSTEM_SHOOTER.aimTilter(() -> SUBSYSTEM_SHOOTER.shooterInterpolator.interpolateAngle(SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker()))
      )
    ).onFalse(SUBSYSTEM_SHOOTER.setTilter(0.0));




   //NoteInConveyerTrigger.or(NoteInFeederTrigger).whileTrue(SUBSYSTEM_SHOOTER.IdleShooter());

    NoteInFeederTrigger.whileTrue(SUBSYSTEM_SHOOTER.IdleShooter());
    NoteInFeederTrigger.whileTrue(SUBSYSTEM_CONVEYER.setConveyerSpeed(0.0));


    //TODO: This will probably break the code, get ready to disable the robot :] !
    //NoteInFeederTrigger.negate().and(ShooterAtHomeTrigger.negate()).onTrue(SUBSYSTEM_SHOOTER.setTilter(0.0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));



    //LockDriveTrigger.onTrue(SUBSYSTEM_SWERVEDRIVE.lockDrive());

    DRIVER_L1.and(NoteInConveyerTrigger.negate()).and(NoteInFeederTrigger.negate()).whileTrue(
      new SequentialCommandGroup(
        SUBSYSTEM_INTAKE.intakeDown(),
        SUBSYSTEM_CONVEYER.intakeGamePiece(),
        SUBSYSTEM_INTAKE.intakeUp()  
      ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    )
    .onFalse(SUBSYSTEM_INTAKE.intakeUp());

    NoteInConveyerTrigger.and(ShooterAtHomeTrigger).onTrue(
      SUBSYSTEM_CONVEYER.setConveyerSpeed(0.3)//.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      .andThen(
        SUBSYSTEM_SHOOTER.loadGamePiece().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      )
      .andThen(
        SUBSYSTEM_CONVEYER.setConveyerSpeed(0.0) //TODO: Does this break the code?
      )
    );

    DRIVER_A.whileTrue(SUBSYSTEM_SHOOTER.shoot(4022,2681, false));

    DRIVER_Y.whileTrue(SUBSYSTEM_SHOOTER.aimTilter( () -> SUBSYSTEM_SHOOTER.shooterInterpolator.interpolateAngle(SUBSYSTEM_SWERVEDRIVE.getDistanceToSpeaker())));

    //DRIVER_X.whileTrue(SUBSYSTEM_SHOOTER.setFeederSpeed(0.6)).onFalse(SUBSYSTEM_SHOOTER.setFeederSpeed(0.0));
    //DRIVER_X.whileTrue(new autoAimSpeaker(SUBSYSTEM_SHOOTER, SUBSYSTEM_SWERVEDRIVE));
    //DRIVER_B.whileTrue(SUBSYSTEM_SWERVEDRIVE.pathFind(new Pose2d(new Translation2d(1.70,5.52),SUBSYSTEM_SWERVEDRIVE.getRotation2d())));

    DRIVER_START.whileTrue(SUBSYSTEM_SWERVEDRIVE.zeroRobotHeading());


    OP_1.whileTrue(SUBSYSTEM_CLIMBER.setWinchSpeed(0.7)).onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0.0));
    //OP_1.onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0.0));
    OP_2.whileTrue(SUBSYSTEM_CLIMBER.setWinchSpeed(-0.7)).onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0));
    //OP_2.onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0.0));

    OP_11.whileTrue(SUBSYSTEM_SHOOTER.zeroTilter(0.0));
    OP_12.whileTrue(SUBSYSTEM_SHOOTER.setTilter(0)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_13.whileTrue(SUBSYSTEM_SHOOTER.setTilter(10)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_14.whileTrue(SUBSYSTEM_SHOOTER.setTilter(20)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_15.whileTrue(SUBSYSTEM_SHOOTER.setTilter(30)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_16.whileTrue(SUBSYSTEM_SHOOTER.setTilter(35)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());


    OP_17.whileTrue(SUBSYSTEM_SHOOTER.setTilter(30.5)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());
    OP_18.whileTrue(SUBSYSTEM_SHOOTER.setTilter(31)).onFalse(SUBSYSTEM_SHOOTER.stopTilter());

    //OP_15.whileTrue(new autoAimSpeaker(SUBSYSTEM_SHOOTER));
   // OP_14.whileTrue(SUBSYSTEM_SHOOTER.setTiltertoManual());



    //SYS ID CONTROLS _________________________________________________________________________________________________________________________________________________________________

    SYSID_1.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistaticLeft(SysIdRoutine.Direction.kForward));
    SYSID_2.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistaticLeft(SysIdRoutine.Direction.kReverse));
    SYSID_3.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamicLeft(SysIdRoutine.Direction.kForward));
    SYSID_4.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamicLeft(SysIdRoutine.Direction.kReverse));

    SYSID_5.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistaticRight(SysIdRoutine.Direction.kForward));
    SYSID_6.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistaticRight(SysIdRoutine.Direction.kReverse));
    SYSID_7.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamicRight(SysIdRoutine.Direction.kForward));
    SYSID_8.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamicRight(SysIdRoutine.Direction.kReverse));

    SYSID_9.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SYSID_10.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SYSID_11.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SYSID_12.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SYSID_13.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdQuasistaticModuleTurning(SysIdRoutine.Direction.kForward));
    SYSID_14.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdQuasistaticModuleTurning(SysIdRoutine.Direction.kForward));
    SYSID_15.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdDynamicModuleTurning(SysIdRoutine.Direction.kForward));
    SYSID_16.whileTrue(SUBSYSTEM_SWERVEDRIVE.sysIdDynamicModuleTurning(SysIdRoutine.Direction.kReverse));

    SYSID_17.whileTrue(SUBSYSTEM_INTAKE.sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(() -> SUBSYSTEM_INTAKE.getIntakePosition() >= Constants.IntakeSubsystemConstants.Forward_IntakePivot_Position));
    SYSID_18.whileTrue(SUBSYSTEM_INTAKE.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(() -> SUBSYSTEM_INTAKE.getIntakePosition() <= Constants.IntakeSubsystemConstants.Reverse_IntakePivot_Position));
    SYSID_19.whileTrue(SUBSYSTEM_INTAKE.sysIdDynamic(SysIdRoutine.Direction.kForward).until(() -> SUBSYSTEM_INTAKE.getIntakePosition() >= Constants.IntakeSubsystemConstants.Forward_IntakePivot_Position));
    SYSID_20.whileTrue(SUBSYSTEM_INTAKE.sysIdDynamic(SysIdRoutine.Direction.kReverse).until(() -> SUBSYSTEM_INTAKE.getIntakePosition() <= Constants.IntakeSubsystemConstants.Reverse_IntakePivot_Position));

    SYSID_21.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistaticTilter(SysIdRoutine.Direction.kForward).until(() -> SUBSYSTEM_SHOOTER.getTilterPosition() >= 60));
    SYSID_22.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistaticTilter(SysIdRoutine.Direction.kReverse).until(() -> SUBSYSTEM_SHOOTER.getTilterPosition() <= 0 ));
    SYSID_23.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamicTiler(SysIdRoutine.Direction.kForward).until(() -> SUBSYSTEM_SHOOTER.getTilterPosition() >= 60));
    SYSID_24.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamicTiler(SysIdRoutine.Direction.kReverse).until(() -> SUBSYSTEM_SHOOTER.getTilterPosition() <= 0 ));

  }

  public Command getAutonomousCommand() {
    //return Autos.simpleFollowPath(SUBSYSTEM_SWERVEDRIVE, "Shop Pickup Note 2");
  //  return Autos.simpleFollowPath(SUBSYSTEM_SWERVEDRIVE, "Test1");
    //return Autos.simpleFollowChoreo(SUBSYSTEM_SWERVEDRIVE, "Test3");
        //return new PathPlannerAuto("TwoNoteAuto");

        //return null;
        return new PathPlannerAuto("FourNoteAutoUnderSpeaker");
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
