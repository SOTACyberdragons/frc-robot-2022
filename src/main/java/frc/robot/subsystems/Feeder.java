// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class Feeder extends SubsystemBase {

//     private WPI_TalonSRX feederMotor;
//     private int feederSpeed = -1;
//     private DigitalInput stopSign = new DigitalInput(RobotMap.STOP_SIGN); //break-beam

//     public boolean getBreakBeam() {
//         return stopSign.get();
//     }

//     public Feeder() {
//         feederMotor = new WPI_TalonSRX(RobotMap.FEEDER_MOTOR);

//     }

//    public void feedIn() {
//        feederMotor.set(feederSpeed);

//    }

//    public void feedOut() {
//        feederMotor.set(-feederSpeed);
//    }

 
//     public void stopFeeding(){
//         feederMotor.set(0);
//     }
// }