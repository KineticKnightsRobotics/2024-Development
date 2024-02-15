package frc.robot.commands;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;


public class setShooterSpeed extends Command {

    Shooter subsystem;
    double percentOutput;

    public setShooterSpeed(double _speed, Shooter _subsystem) {
        addRequirements(_subsystem);
        subsystem = _subsystem;
        percentOutput = _speed;
    }

    @Override
    public void initialize() {
        subsystem.setShooterSpeed(percentOutput);
        subsystem.setFeederSpeed(percentOutput);
    }
    @Override
    public void execute() {
        /*
        if (subsystem.getShooterFRPM() > speed && subsystem.getShooterLRPM() > speed) {
            subsystem.toggleShooterBlock(DoubleSolenoid.Value.kReverse);
            subsystem.setFeederSpeed(0.8);
        }
        else {
            subsystem.toggleShooterBlock(DoubleSolenoid.Value.kForward);
            //asubsystem.setFeederSpeed(0.0);
        }
        */
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.setShooterSpeed(0.0);
        subsystem.toggleShooterBlock(DoubleSolenoid.Value.kForward);
        subsystem.setFeederSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
