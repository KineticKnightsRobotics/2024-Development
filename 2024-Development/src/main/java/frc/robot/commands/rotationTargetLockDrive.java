package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.geometry.Translation2d;

//import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.lib.Constants.SwerveSubsystemConstants;
import frc.robot.lib.PID_Config.RotationTargetLock;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;


public class rotationTargetLockDrive extends Command {

    private final SwerveDrive subsystem;
    private final DoubleSupplier SUPPLIER_xSpeed;
    private final DoubleSupplier SUPPLIER_ySpeed;
    private final BooleanSupplier SUPPLIER_Field_Oriented;
    private final BooleanSupplier flipControls;
    private final DoubleSupplier SUPPLIER_Period;
private PIDController rotationPID = new PIDController(RotationTargetLock.Proportional,RotationTargetLock.Integral,RotationTargetLock.Derivitive);
    //private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public rotationTargetLockDrive(
        SwerveDrive m_subsystem,
        DoubleSupplier xSpeed, 
        DoubleSupplier ySpeed, 
        DoubleSupplier zSpeed,
        BooleanSupplier fieldOriented,
        BooleanSupplier _flipControls,
        DoubleSupplier timePeriod
        ){
        subsystem = m_subsystem;
        SUPPLIER_xSpeed = xSpeed;
        SUPPLIER_ySpeed = ySpeed;
        SUPPLIER_Field_Oriented = fieldOriented;
        flipControls = _flipControls;
        SUPPLIER_Period = timePeriod;
        addRequirements(subsystem);
        //this.xLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_SPEED);
        //this.yLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_SPEED);
        //this.zLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_TURN);
        rotationPID.setTolerance(2, 5);
        rotationPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
                // double[] positionData = LimelightHelpers.getTargetPose_RobotSpace("limelight");
                //  double desiredAngle = positionData[5];

        double joystickX = SUPPLIER_xSpeed.getAsDouble();
        double joystickY = SUPPLIER_ySpeed.getAsDouble();

        if (flipControls.getAsBoolean()) {
            joystickX *= -1;
            joystickY *= -1;
        }

        double xSpeed   = ((joystickX * joystickX) * (joystickX<0 ? -1 : 1)) *   SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;     // * 0.2;
        double ySpeed   = ((joystickY * joystickX) * (joystickY<0 ? -1 : 1)) *   SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;     // * 0.2; //Determine new velocity
        double rotSpeed = rotationPID.calculate(subsystem.getRotation2d().getDegrees(), subsystem.getRotationRelativeToSpeaker().getDegrees()+180.0);
        boolean fieldRelative = SUPPLIER_Field_Oriented.getAsBoolean();
        double timePeriod = SUPPLIER_Period.getAsDouble();

        //ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, subsystem.getRotation2d());
      
        ChassisSpeeds chassisSpeed = ChassisSpeeds.discretize(
            fieldRelative 
                ? 
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, subsystem.getRotation2d())
                : 
                    new ChassisSpeeds(xSpeed, ySpeed, rotSpeed),
            timePeriod);
     
        
        subsystem.setChassisSpeed(chassisSpeed,true);

        /*
        if (subsystem.idle_Timer_Lock.get() > 0.5) {
            subsystem.lockChassis();
            subsystem.idle_Timer_Lock.reset();
        }
        */
    }
    @Override
    public void end(boolean interrupted) {
        //this.swerve.drive(new Translation2d(0, 0), 0, true, false);
        //PLEASE SET THIS FOR SAFETY!!!
        //this.swerve.stopMotors();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
