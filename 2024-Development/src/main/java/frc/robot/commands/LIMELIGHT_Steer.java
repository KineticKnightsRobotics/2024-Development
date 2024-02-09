package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.LimeLight;
import frc.robot.lib.PID_Config;
//import frc.robot.lib.Constants.Kinematics;
import frc.robot.subsystems.SwerveDrive;

public class LIMELIGHT_Steer extends Command {

    static SwerveDrive SUBSYSTEM_SWERVEDRIVE;
    static LimeLight SUBSYSTEM_LIMELIGHT;
    static PIDController TurnPID;
    
    double LL_Angle;
    double currentAngle;
    double targetAngle;
    ChassisSpeeds targetState;


    public LIMELIGHT_Steer(SwerveDrive drive, LimeLight limeLight){
        addRequirements(drive,limeLight);
        SUBSYSTEM_LIMELIGHT = limeLight;
        SUBSYSTEM_SWERVEDRIVE = drive;

        TurnPID = new PIDController(
            PID_Config.VisionDriving.Steering.Proportional,
            PID_Config.VisionDriving.Steering.Integral,
            PID_Config.VisionDriving.Steering.Derivitive
        );
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        LL_Angle = SUBSYSTEM_LIMELIGHT.getLimeLightTX();
        currentAngle = SUBSYSTEM_SWERVEDRIVE.getRobotHeading();
        targetAngle = currentAngle + LL_Angle;
        targetState = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,0.0,TurnPID.calculate(currentAngle,targetAngle),SUBSYSTEM_SWERVEDRIVE.getRotation2d());
        SUBSYSTEM_SWERVEDRIVE.setAutoChassisSpeed(targetState);
    }

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished(){
        return false;
    }
}