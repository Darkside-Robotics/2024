package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbingKook extends SubsystemBase{
    CANSparkMax ClimbingMotor;

    public ClimbingKook() {
        ClimbingMotor = new CANSparkMax(14, MotorType.kBrushless);

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
