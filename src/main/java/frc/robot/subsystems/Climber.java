package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static TalonSRX leftMotor;
    private static TalonSRX rightMotor;
    
    public Climber()
    {
        leftMotor = new TalonSRX(10);
        rightMotor = new TalonSRX(11);

        leftMotor.follow(rightMotor);
    }

    public void moveUp()
    {
        rightMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void moveDown()
    {
        rightMotor.set(ControlMode.PercentOutput, -0.5);
    }
    
    public void stop() { 
        rightMotor.set(ControlMode.PercentOutput, 0);
    }
}
