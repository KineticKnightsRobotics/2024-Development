// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.lib.Constants.IntakeSubsystemConstants;
import frc.robot.lib.Constants.OIConstants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;



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
  //private final Climber SUBSYSTEM_CLIMBER = new Climber();
  private final SwerveDrive SUBSYSTEM_SWERVEDRIVE = new SwerveDrive();
  private Command lockCommand = new RunCommand(() -> SUBSYSTEM_SWERVEDRIVE.lockChassis(),SUBSYSTEM_SWERVEDRIVE);


  private final static CommandJoystick JOYSTICK_DRIVER = new CommandJoystick(OIConstants.ID_CONTROLLER_DRIVER);

  Trigger DRIVER_A = new Trigger(JOYSTICK_DRIVER.button(1));
  Trigger DRIVER_B = new Trigger(JOYSTICK_DRIVER.button(2));
  Trigger DRIVER_X = new Trigger(JOYSTICK_DRIVER.button(3));
  Trigger DRIVER_Y = new Trigger(JOYSTICK_DRIVER.button(4));
  Trigger DRIVER_L1= new Trigger(JOYSTICK_DRIVER.button(5));
  Trigger DRIVER_R1= new Trigger(JOYSTICK_DRIVER.button(6));
  Trigger DRIVER_START= new Trigger(JOYSTICK_DRIVER.button(7));
  Trigger DRIVER_BACK = new Trigger(JOYSTICK_DRIVER.button(8));
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

  


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    SUBSYSTEM_SWERVEDRIVE.setDefaultCommand(
      new joystickDrive(
        SUBSYSTEM_SWERVEDRIVE, 
        () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
        () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
        () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
        () -> true, 
        () -> 0.02
      )
    );

    NamedCommands.registerCommand("ResetModulePosition", SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());
    NamedCommands.registerCommand("IntakeDown" , SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Forward_IntakePivot_Position));
    NamedCommands.registerCommand("IntakeUp" , SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Reverse_IntakePivot_Position));
    NamedCommands.registerCommand("AutoConveyer", new intakeLineBreak(SUBSYSTEM_CONVEYER,SUBSYSTEM_INTAKE));
    NamedCommands.registerCommand("AutoAimSpeaker", new autoAimSpeaker(SUBSYSTEM_SHOOTER));
    NamedCommands.registerCommand("AutoRunShooter", new autoRunShooter(SUBSYSTEM_SHOOTER,SUBSYSTEM_CONVEYER));
    NamedCommands.registerCommand("AutoSetShooterIdle", new autoSetShooterIdle(SUBSYSTEM_SHOOTER));
    NamedCommands.registerCommand("AutoLoadShooter", new autoLoadShooter(SUBSYSTEM_CONVEYER,SUBSYSTEM_SHOOTER));
    NamedCommands.registerCommand("ShooterDown", SUBSYSTEM_SHOOTER.setTilter(0.0));


    NamedCommands.registerCommand("CapturePiece", 
      new SequentialCommandGroup(
          SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Forward_IntakePivot_Position),
          new intakeLineBreak(SUBSYSTEM_CONVEYER, SUBSYSTEM_INTAKE),
          SUBSYSTEM_INTAKE.setIntakePosition(IntakeSubsystemConstants.Reverse_IntakePivot_Position)
    //intake down, feed note to panel, set intake up
    ));

    
    // Configure the trigger bindings
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser("Straight");

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

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

    rightTrigger().whileTrue( new PIDCommand(
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

      DRIVER_BACK.whileTrue(lockCommand);

    DRIVER_A.whileTrue(new runShooter_OpenLoop(3200, SUBSYSTEM_SHOOTER));
    //DRIVER_A.whileTrue(new SHOOTER_runShooter_ClosedLoop(4300, SUBSYSTEM_SHOOTER));

    DRIVER_X.onTrue(
        new SequentialCommandGroup(
          //new CONVEYERSHOOTER_loadFeeder(SUBSYSTEM_CONVEYER, SUBSYSTEM_SHOOTER,SUBSYSTEM_INTAKE)
          new runConveyerlLineBreak(SUBSYSTEM_CONVEYER)
        
        //new SHOOTER_moveFeederDistance(SUBSYSTEM_SHOOTER, -20),
        //new SHOOTER_runShooter(0, SUBSYSTEM_SHOOTER)
        )
    );

    DRIVER_Y.whileTrue(
      new runFeeder(0.8, SUBSYSTEM_SHOOTER)
    );

    DRIVER_START.whileTrue(SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());

    //DRIVER_BACK.onTrue(SUBSYSTEM_SWERVEDRIVE.lockDrive());


    //OP_1.onTrue(new SHOOTER_moveFeederDistance(SUBSYSTEM_SHOOTER, -20));

    OP_2.whileTrue(
      new SequentialCommandGroup(
        new loadFeeder(SUBSYSTEM_CONVEYER, SUBSYSTEM_SHOOTER,SUBSYSTEM_INTAKE),
        new moveFeederDistance(SUBSYSTEM_SHOOTER, -20),
        new runShooter_OpenLoop(0, SUBSYSTEM_SHOOTER)
        )
    );




    OP_1.whileTrue(new autoLoadShooter(SUBSYSTEM_CONVEYER, SUBSYSTEM_SHOOTER));


    OP_11.onTrue(SUBSYSTEM_SHOOTER.zeroTilter(0.0));

    OP_12.onTrue(SUBSYSTEM_SHOOTER.setTilter(0.0));

    OP_15.whileTrue(new autoAimSpeaker(SUBSYSTEM_SHOOTER));

    OP_14.onTrue(SUBSYSTEM_SHOOTER.setTilter(30.0));

    OP_16.onTrue(SUBSYSTEM_SHOOTER.setTilter(50.0));

    //OP_19.whileTrue(SUBSYSTEM_SWERVEDRIVE.resetDriveOdemeter(pose));


    ///OP_19.onTrue(SUBSYSTEM_CLIMBER.setWinchSpeed(0.8));
    //OP_19.onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0.0));
    //OP_20.onTrue(SUBSYSTEM_CLIMBER.setWinchSpeed(-0.8));
    //OP_20.onFalse(SUBSYSTEM_CLIMBER.setWinchSpeed(0.0));

    SYSID_1.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SYSID_2.whileTrue(SUBSYSTEM_SHOOTER.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SYSID_3.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SYSID_4.whileTrue(SUBSYSTEM_SHOOTER.sysIdDynamic(SysIdRoutine.Direction.kReverse));



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

  public static boolean DRIVER_RT() {
    return JOYSTICK_DRIVER.getRawAxis(3) > 0.5;
  }
  public static Trigger rightTrigger(){
  return new Trigger(()-> JOYSTICK_DRIVER.getRawAxis(3)>=0.3);
}
}
