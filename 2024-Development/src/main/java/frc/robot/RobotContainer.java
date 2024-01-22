// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.lib.LimeLight;

import frc.robot.lib.Constants.OIConstants;
import frc.robot.lib.Constants.POIGeometryConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrive SUBSYSTEM_SWERVEDRIVE = new SwerveDrive();
  private final LimeLight SUBSYSTEM_LIMELIGHT = new LimeLight();
  private final Intake SUBSYSTEM_INTAKE = new Intake();
  private final Conveyer SUBSYSTEM_CONVEYER = new Conveyer();

  private final CommandJoystick JOYSTICK_DRIVER = new CommandJoystick(OIConstants.ID_CONTROLLER_DRIVER);

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

  private final CommandJoystick JOYSTICK_OPERATOR = new CommandJoystick(OIConstants.ID_CONTROLLER_OPERATOR);

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putData(SUBSYSTEM_LIMELIGHT);
    SmartDashboard.putData(SUBSYSTEM_SWERVEDRIVE);

    SUBSYSTEM_SWERVEDRIVE.setDefaultCommand(
      new DRIVE_JoystickDrive(
        SUBSYSTEM_SWERVEDRIVE, 
        () -> -JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_X), 
        () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y), 
        () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Z), 
        () -> true)
        
    );
    // Configure the trigger bindings
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("I hate big chungus");

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    DRIVER_B.onTrue(SUBSYSTEM_SWERVEDRIVE.zeroRobotHeading());

    DRIVER_L1.whileTrue(new LIMELIGHT_Steer(SUBSYSTEM_SWERVEDRIVE, SUBSYSTEM_LIMELIGHT));
    DRIVER_R1.whileTrue(new LIMELIGHT__Strafe(SUBSYSTEM_LIMELIGHT,SUBSYSTEM_SWERVEDRIVE,POIGeometryConstants.Test1.OFFSET_POI_X,() -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y)));


    OP_1.whileTrue(SUBSYSTEM_SWERVEDRIVE.startTrajectory());
    OP_2.onTrue(SUBSYSTEM_SWERVEDRIVE.zeroRoboOdemetry());

    OP_4.whileTrue(new INTAKE_SetRollerSpeed(SUBSYSTEM_INTAKE, 0.4));

    OP_5.onTrue(SUBSYSTEM_INTAKE.toggleSolenoids(false));
    OP_6.onTrue(SUBSYSTEM_INTAKE.toggleSolenoids(true));
    OP_7.whileTrue(
      new SequentialCommandGroup(
        SUBSYSTEM_INTAKE.toggleSolenoids(true),
        new INTAKE_SetRollerSpeed(SUBSYSTEM_INTAKE, 0.4)
      )
    );

    OP_8.whileTrue(SUBSYSTEM_CONVEYER.runConveyer(0.3));
    
  }

  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return  new PathPlannerAuto("I hate big chungus");
  // }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  } 
}
