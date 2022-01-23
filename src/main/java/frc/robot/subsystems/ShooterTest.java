package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.OI;
import frc.robot.commands.ShootTest;


public class ShooterTest extends SubsystemBase {
    private static TalonSRX leftMotor;
    private static TalonSRX rightMotor;

    OI oi = new OI();
    
    public ShooterTest()
    {
        leftMotor = new TalonSRX(10);
        rightMotor = new TalonSRX(11);
    }

    public void givePower()
    {
        if (oi.getShootButtonState()) {
            leftMotor.set(ControlMode.PercentOutput, 1);
        }
    }
}
