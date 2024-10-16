package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.Constants.SwerveSubsystemConstants;
import frc.robot.lib.PID_Config.RotationTargetLock;
import frc.robot.subsystems.SwerveDrive;

public class speakerAiming extends Command {
    
    private final SwerveDrive subsystem;
    private final DoubleSupplier SUPPLIER_xSpeed;
    private final DoubleSupplier SUPPLIER_ySpeed;
    private final DoubleSupplier SUPPLIER_Period;


    private PIDController rotationPID = new PIDController(RotationTargetLock.Proportional,RotationTargetLock.Integral,RotationTargetLock.Derivitive);

    //private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public speakerAiming(
        SwerveDrive m_subsystem,
        DoubleSupplier xSpeed, 
        DoubleSupplier ySpeed,
        DoubleSupplier timePeriod
        ){
        subsystem = m_subsystem;
        SUPPLIER_xSpeed = xSpeed;
        SUPPLIER_ySpeed = ySpeed;
        SUPPLIER_Period = timePeriod;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double joystickX = SUPPLIER_xSpeed.getAsDouble();
        double joystickY = SUPPLIER_ySpeed.getAsDouble(); //grab speeds and apply deadband
        var alliance = DriverStation.getAlliance();

        if (alliance.get() == DriverStation.Alliance.Red) {
            joystickX *= -1;
            joystickY *= -1;
        }       
        double xSpeed   = ((joystickX * joystickX) * (joystickX<0 ? -1 : 1)) *    SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;// * 0.2;
        double ySpeed   = ((joystickY * joystickY) * (joystickY<0 ? -1 : 1)) *    SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;

        double timePeriod = SUPPLIER_Period.getAsDouble();
         
        subsystem.setChassisSpeed(createChassisSpeed(xSpeed, ySpeed),true);
    }
    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    private Rotation2d calculateHeadingDelta() {
        // Check which side of speaker we're on
        
        if (subsystem.getPose().getY() > subsystem.getSpeakerPose().get().getY() + 0.5) {
                //Robot is to the right of the speaker
                return subsystem.getRotationRelativeToSpeaker(-0.1); //return subsystem.getRotationRelativeToPoint(3.0, -0.1);
            }
        else if (subsystem.getPose().getY() < subsystem.getSpeakerPose().get().getY() - 0.5) {
                //Robot is to the left of the speaker
                return subsystem.getRotationRelativeToSpeaker(0.1);//return subsystem.getRotationRelativeToPoint(3.0, 0.1);
            } 
        else {
            return subsystem.getRotationRelativeToSpeaker(0.0); // normal old aiming
        }
    }

    private ChassisSpeeds createChassisSpeed(double xSpeed, double ySpeed) {
        return ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotationPID.calculate(subsystem.getRotation2d().getDegrees(), calculateHeadingDelta().getDegrees()+180),
                subsystem.getRotation2d()
                ),
                SUPPLIER_Period.getAsDouble()
            );
    }



}
