// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class Intake extends SubsystemBase {

//     private WPI_TalonSRX intakeMotor;
//     private DoubleSolenoid intakeSolenoid;
//     private int intakeSpeed = 1;

//     public Intake() {
//         DoubleSolenoid testSolenoid;
       
       
//         intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);
//         intakeSolenoid = new DoubleSolenoid(null, RobotMap.DOUBLE_SOLENOID_ZERO,RobotMap.DOUBLE_SOLENOID_ONE);

//     }

//    public void setIntakeSpeed(double intakeSpeed) {
//        intakeMotor.set(intakeSpeed);

//    }

 
//     public void stopMoving(){
//         intakeMotor.set(0);
//     }

//     public void moveIntake(String direction) {
//         if(direction == "down") {
//             intakeSolenoid.set(Value.kForward);
//         } else if(direction == "up") {
//             intakeSolenoid.set(Value.kReverse);
//         }
//     }
// }