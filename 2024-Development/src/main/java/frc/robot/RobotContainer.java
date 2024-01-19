// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.lib.LimeLight;

import frc.robot.lib.Constants.OIConstants;
import frc.robot.lib.Constants.POIGeometryConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrive SUBSYSTEM_SWERVEDRIVE = new SwerveDrive();
  private final LimeLight SUBSYSTEM_LIMELIGHT = new LimeLight();

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
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));


    //DRIVER_A.onTrue(SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());
    DRIVER_B.onTrue(SUBSYSTEM_SWERVEDRIVE.zeroRobotHeading());

    DRIVER_X.whileTrue(new LIMELIGHT_Steer(SUBSYSTEM_SWERVEDRIVE, SUBSYSTEM_LIMELIGHT));

    //DRIVER_START.onTrue(SUBSYSTEM_LIMELIGHT.changePipeline(0));
    //DRIVER_BACK.onTrue(SUBSYSTEM_LIMELIGHT.changePipeline(1));

    //DRIVER_Y.whileTrue(new RunShooter(SUBSYSTEM_CONVEYER));

    DRIVER_L1.whileTrue(
      new LIMELIGHT_Steer(SUBSYSTEM_SWERVEDRIVE, SUBSYSTEM_LIMELIGHT)
    );
    DRIVER_R1.whileTrue(
      new LIMELIGHT__Strafe(
       SUBSYSTEM_LIMELIGHT,
       SUBSYSTEM_SWERVEDRIVE,
       POIGeometryConstants.Test1.OFFSET_POI_X,
       () -> JOYSTICK_DRIVER.getRawAxis(OIConstants.CONTROLLER_DRIVER_Y)

      )
    );
    OP_1.whileTrue(SUBSYSTEM_SWERVEDRIVE.startTrajectory());
    OP_2.onTrue(SUBSYSTEM_SWERVEDRIVE.zeroRoboOdemetry());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
