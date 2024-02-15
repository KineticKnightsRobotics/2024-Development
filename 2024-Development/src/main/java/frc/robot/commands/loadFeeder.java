package frc.robot.commands;


import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class CONVEYERSHOOTER_loadFeeder extends Command {


    Conveyer m_Conveyer;
    Shooter m_Shooter;
    Intake m_Intake;

    int timer;

    public CONVEYERSHOOTER_loadFeeder(Conveyer _Conveyer, Shooter _Shooter, Intake _Intake) {
        addRequirements(_Conveyer, _Shooter);

        m_Conveyer = _Conveyer;
        m_Shooter = _Shooter;
        m_Intake = _Intake;
        timer = 0;
    }

    @Override
    public void initialize() {
        m_Intake.setRollerSpeed(0.8);
        m_Conveyer.setConveyerSpeed(0.2);
        m_Shooter.setFeederSpeed(0.8);
    }

    @Override
    public void execute() {
        if ( ! m_Conveyer.getLineBreak()) {

            timer +=1;
            if (timer > 3) {m_Shooter.setFeederSpeed(0.0);}
            m_Shooter.setFeederSpeed(timer);
        }
        else {m_Shooter.setFeederSpeed(0.8);}
    }
    @Override
    public void end(boolean interrupted) {
        m_Shooter.setFeederSpeed(0);
        m_Conveyer.setConveyerSpeed(0);
        m_Intake.setRollerSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return (! m_Conveyer.getLineBreak()) && timer > 60;
    }
    
}
