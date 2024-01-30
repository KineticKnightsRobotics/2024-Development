package frc.robot.commands;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;


public class SHOOTER_runShooter extends Command {

    Shooter subsystem;
    double speed;

    public SHOOTER_runShooter(double _speed, Shooter _subsystem) {
        addRequirements(_subsystem);
        subsystem = _subsystem;
        speed = _speed;
    }

    @Override
    public void initialize() {
        subsystem.setShooterSpeed(speed);
    }
    @Override
    public void execute() {
        if (subsystem.getShooterFRPM() > 4600) {
            subsystem.toggleShooterBlock(DoubleSolenoid.Value.kReverse);
            subsystem.setFeederSpeed(speed);
        }
        else {
            subsystem.toggleShooterBlock(DoubleSolenoid.Value.kForward);
            subsystem.setFeederSpeed(0.0);
        }
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
