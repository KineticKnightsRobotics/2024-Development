package frc.robot.commands;


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
    public void end(boolean interrupted) {
        subsystem.setShooterSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
