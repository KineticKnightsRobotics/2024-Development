package frc.robot.commands;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;


public class SHOOTER_runShooter_ClosedLoop extends Command {

    Shooter subsystem;
    double speed;

    public SHOOTER_runShooter_ClosedLoop(double desiredRPM, Shooter _subsystem) {
        addRequirements(_subsystem);
        subsystem = _subsystem;
        speed = desiredRPM;
    }

    @Override
    public void initialize() {
        subsystem.setShooterRPM(speed);
    }
    @Override
    public void execute() {
        if (subsystem.getShooterFRPM() > speed && subsystem.getShooterLRPM() > speed && (Math.abs(subsystem.getShooterFRPM() - subsystem.getShooterLRPM()) <= 100 ) ) {
            subsystem.toggleShooterBlock(DoubleSolenoid.Value.kReverse);
            subsystem.setFeederSpeed(0.8);
        }
        else {
            subsystem.toggleShooterBlock(DoubleSolenoid.Value.kForward);
        }
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.setShooterRPM(0.0);
        subsystem.toggleShooterBlock(DoubleSolenoid.Value.kForward);
        subsystem.setFeederSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
