package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ShooterSubsystem extends SubsystemBase {
    SparkMax LeftShootMotor;
    SparkMax RightShootMotor;
    SparkMax IntakeMotor;
    SparkMax SharedMotor;
    Spark blinkinLed;
    boolean loaded;
    final DigitalInput beamBreak = new DigitalInput(0);


    
    public ShooterSubsystem(){
        LeftShootMotor = new SparkMax(12, MotorType.kBrushless);
        RightShootMotor = new SparkMax(13, MotorType.kBrushless);
        IntakeMotor = new SparkMax(NoteConstants.IntakeMotorID, MotorType.kBrushless);
        SharedMotor = new SparkMax(NoteConstants.SharedMotorID, MotorType.kBrushless);
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
