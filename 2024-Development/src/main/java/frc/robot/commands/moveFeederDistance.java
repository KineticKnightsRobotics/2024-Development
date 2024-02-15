package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

import frc.robot.lib.PID_Config.ShooterSubsystem.FeederPositionKinematicsPID;;

public class moveFeederDistance extends Command {

    Shooter m_Shooter;

    double startPoint, endPoint, rotations, speed;

    PIDController controller;
    
    public moveFeederDistance(Shooter _subsystem, double _rotations) {
        addRequirements(_subsystem);
        m_Shooter = _subsystem;
        rotations = _rotations;
        

        controller = new PIDController(
            FeederPositionKinematicsPID.Proportional,
            FeederPositionKinematicsPID.Integral,
            FeederPositionKinematicsPID.Derivitive
        );
    }

    @Override
    public void initialize() {



        startPoint = m_Shooter.getFeedPostition();
        endPoint = startPoint + rotations;
    }
    @Override
    public void execute() {
        //feeder running forwards
        speed = controller.calculate(m_Shooter.getFeedPostition(),endPoint);

        m_Shooter.setFeederSpeed(speed);
    }
    @Override
    public void end(boolean interrupted) {
        m_Shooter.setFeederSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return m_Shooter.getFeedPostition() == endPoint;
    }
}