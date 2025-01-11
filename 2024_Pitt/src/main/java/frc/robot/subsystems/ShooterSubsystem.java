package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ShooterSubsystem extends SubsystemBase {
    CANSparkMax LeftShootMotor;
    CANSparkMax RightShootMotor;
    CANSparkMax IntakeMotor;
    CANSparkMax SharedMotor;
    Spark blinkinLed;
    boolean loaded;
    final DigitalInput beamBreak = new DigitalInput(0);


    
    public ShooterSubsystem(){
        LeftShootMotor = new CANSparkMax(12, MotorType.kBrushless);
        RightShootMotor = new CANSparkMax(13, MotorType.kBrushless);
        IntakeMotor = new CANSparkMax(NoteConstants.IntakeMotorID, MotorType.kBrushless);
        SharedMotor = new CANSparkMax(NoteConstants.SharedMotorID, MotorType.kBrushless);
        blinkinLed = new Spark(9);

    }

    

    public void StartShooter(){
        LeftShootMotor.set(-1);
        RightShootMotor.set(1);
        SharedMotor.set(0.4);

    }

    public void StopShooter(){
        LeftShootMotor.set(0);
        RightShootMotor.set(0);
        SharedMotor.set(0);
    }

    public void runIntake(){
        if (!beamBreak.get()) {
            stopIntake();
            loaded = true;
        }
        else{
            LeftShootMotor.set(0.1);
            RightShootMotor.set(-0.1);
            IntakeMotor.set(-1);
            SharedMotor.set(0.3);

            loaded = false;
        }
    }

    public void shootamp(){
        LeftShootMotor.set(-0.1313);
        RightShootMotor.set(0.1313);
        SharedMotor.set(0.4);
    }

    public void stopIntake(){
        IntakeMotor.set(0);
        SharedMotor.set(0);
        LeftShootMotor.set(0);
        RightShootMotor.set(0);
    }

    public boolean noteloaded(){
        blinkinLed.set(0.73);
        return loaded;
    }

    public void notefired (){
        blinkinLed.set(0.61);
    }

    
}
