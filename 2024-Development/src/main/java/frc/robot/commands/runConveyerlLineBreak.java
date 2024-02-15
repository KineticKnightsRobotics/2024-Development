package frc.robot.commands;




import frc.robot.subsystems.Conveyer;
import edu.wpi.first.wpilibj2.command.Command;

// Default command, run at 10% speed unless line break.
public class runConveyerlLineBreak extends Command {
    

    Conveyer m_Conveyer;

    public runConveyerlLineBreak(Conveyer _Conveyer) {
        addRequirements(_Conveyer);
        m_Conveyer = _Conveyer;
    }

    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        if (m_Conveyer.getLineBreak()) {
            m_Conveyer.setConveyerSpeed(0.4);
            
        }
        else {
            m_Conveyer.setConveyerSpeed(0.0);
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_Conveyer.setConveyerSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
