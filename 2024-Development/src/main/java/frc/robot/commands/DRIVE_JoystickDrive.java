package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DRIVE_JoystickDrive extends Command {

    private final SwerveDrive subsystem;
    private final DoubleSupplier SUPPLIER_xSpeed;
    private final DoubleSupplier SUPPLIER_ySpeed;
    private final DoubleSupplier SUPPLIER_zSpeed;
    //private final BooleanSupplier SUPPLIER_Field_Oriented;

    //private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public DRIVE_JoystickDrive(
        SwerveDrive m_subsystem,
        DoubleSupplier xSpeed, 
        DoubleSupplier ySpeed, 
        DoubleSupplier zSpeed,
        BooleanSupplier Field_Oriented
        ){
        this.subsystem = m_subsystem;
        this.SUPPLIER_xSpeed = xSpeed;
        this.SUPPLIER_ySpeed = ySpeed;
        this.SUPPLIER_zSpeed = zSpeed;
        //this.SUPPLIER_Field_Oriented = Field_Oriented;
        addRequirements(subsystem);

        //this.xLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_SPEED);
        //this.yLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_SPEED);
        //this.zLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.LIMIT_SOFT_ACCELERATION_TURN);
    }

    @Override
    public void execute() {


        //Get joystick input from double suppliers
        double xSpeed = SUPPLIER_xSpeed.getAsDouble() * Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * 0.2 * (Math.abs(SUPPLIER_xSpeed.getAsDouble()) > 0.1 ? 1.0 : 0.0);
        double ySpeed = SUPPLIER_ySpeed.getAsDouble() * Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * 0.2 * (Math.abs(SUPPLIER_ySpeed.getAsDouble()) > 0.1 ? 1.0 : 0.0);
        double rotSpeed = SUPPLIER_zSpeed.getAsDouble()* Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN * 0.2;
        

        

        //apply Slew Rate Limiters
        //xSpeed = xLimiter.calculate(xSpeed);
        //xSpeed = yLimiter.calculate(ySpeed);
        //rotSpeed = zLimiter.calculate(rotSpeed);

        //set chassis speeds depending on what orientation the robot is in
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, subsystem.getRotation2d());
        subsystem.setChassisSpeed(chassisSpeed);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
