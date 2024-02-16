// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(zExampleSubsystem subsystem) {
    //return Commands.sequence(subsystem.exampleMethodCommand(), new zExampleCommand(subsystem));
    return null;
  }
  /*
   *     public Command followPath(String pathName) {

        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        Pose2d initialPose = path.getPreviewStartingHolonomicPose();
   */

  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
