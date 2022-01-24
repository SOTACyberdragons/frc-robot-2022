package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterTest extends SubsystemBase {
    private static TalonSRX leftMotor;
    private static TalonSRX rightMotor;
    
    public ShooterTest()
    {
        leftMotor = new TalonSRX(10);
        rightMotor = new TalonSRX(11);
    }

    public void givePower()
    {
        leftMotor.set(ControlMode.PercentOutput, 0.5);
    }
    
    public void stop() { 
        leftMotor.set(ControlMode.PercentOutput, 0);
        rightMotor.set(ControlMode.PercentOutput, 0);
    }
}
