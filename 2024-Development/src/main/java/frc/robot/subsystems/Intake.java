package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.lib.Constants.IntakeSubsystemConstants;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax rollerMotor = new CANSparkMax(9, MotorType.kBrushless);

    DoubleSolenoid solenoidIntakeLeft = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        IntakeSubsystemConstants.IntakePneumatics.CHANNEL_LEFT_FORWARD, 
        IntakeSubsystemConstants.IntakePneumatics.CHANNEL_LEFT_REVERSE
    );

    DoubleSolenoid solenoidIntakeRight = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        IntakeSubsystemConstants.IntakePneumatics.CHANNEL_RIGHT_FORWARD, 
        IntakeSubsystemConstants.IntakePneumatics.CHANNEL_RIGHT_REVERSE
    );

    Compressor airCompressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public Intake(){
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setOpenLoopRampRate(2.0);
        rollerMotor.setInverted(false);

        solenoidIntakeLeft.set (DoubleSolenoid.Value.kReverse);
        solenoidIntakeRight.set(DoubleSolenoid.Value.kReverse);

        airCompressor.enableDigital();
    }

    public void setSolenoids(boolean state){
        //convert boolean to forward/reverse double solenoid values
        DoubleSolenoid.Value newState = (state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        if (solenoidIntakeLeft.get() != newState){solenoidIntakeLeft.set(newState);}
        if (solenoidIntakeRight.get() != newState){solenoidIntakeRight.set(newState);}
    }
    public void setRollerSpeed(double percentOutput) {
        rollerMotor.set(percentOutput);
    }

    public Command toggleSolenoids(boolean state){
        return Commands.runOnce(() -> setSolenoids(state));
    }

}
