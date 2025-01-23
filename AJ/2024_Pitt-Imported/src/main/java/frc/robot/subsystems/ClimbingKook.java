package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;



public class ClimbingKook extends SubsystemBase{
    SparkMax ClimbingMotor;

    public ClimbingKook() {
       // ClimbingMotor = new SparkMax(14, MotorType.kBrushless);

    } 
    
    public void ClimbUp(){
        ClimbingMotor.set(1);


    }

    public void ClimbDown(){
        ClimbingMotor.set(-1);

    }

    public void StopHammerTime(){
        ClimbingMotor.set(0);
    }


}
