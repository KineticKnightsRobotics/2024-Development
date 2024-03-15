package frc.robot.commands.unused;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Intake;

public class intakeLineBreak extends Command{
    Conveyer m_Conveyer;Intake m_Intake;
    public intakeLineBreak(Conveyer m_subsystem, Intake m_subsystem2) {
        addRequirements(m_subsystem);
        m_Conveyer = m_subsystem;
        m_Intake = m_subsystem2;
    }
    @Override
    public void initialize() {
        m_Conveyer.setConveyerSpeed(0.7);
        m_Intake.setRollerSpeed(0.8);
    }
    @Override
    public void end(boolean interrupted) {
        //m_Conveyer.setConveyerSpeed(0.0);
        m_Intake.setRollerSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return m_Conveyer.getLineBreak();
    }

    @Override 
    public void execute(){
         m_Conveyer.setConveyerSpeed(0.7);
        m_Intake.setRollerSpeed(0.8);
    }
    

}
