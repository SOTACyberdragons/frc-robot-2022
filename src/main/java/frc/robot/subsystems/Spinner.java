// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.ColorMatch;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorSensorV3;

// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class Spinner extends SubsystemBase {

//     private WPI_TalonSRX spinnerMotor;

//     private final I2C.Port i2cPort = I2C.Port.kOnboard;
//     private final ColorSensorV3 m_colorSensor;
//     private final ColorMatch  m_colorMatcher = new ColorMatch();

//     //@TODO: configure colors
//     private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
//     private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
//     private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
//     private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


//     String colorString;
//     ColorMatchResult match;


//     public Spinner() {
        
//         m_colorSensor = new ColorSensorV3(i2cPort);
             

//         m_colorMatcher.addColorMatch(kBlueTarget);
//         m_colorMatcher.addColorMatch(kGreenTarget);
//         m_colorMatcher.addColorMatch(kRedTarget);
//         m_colorMatcher.addColorMatch(kYellowTarget);
       
//         spinnerMotor = new WPI_TalonSRX(RobotMap.SPINNER_MOTOR);

//     }

//     public double getBlue() {
//         return m_colorSensor.getColor().blue;
//     }
//     public double getRed() {
//         return m_colorSensor.getColor().red;
//     }
//     public double getGreen() {
//         return m_colorSensor.getColor().green;
//     }
//         public String getColor() {
//         match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
//         if (match.color == kBlueTarget) {
//             colorString = "Blue";
//           } else if (match.color == kRedTarget) {
//             colorString = "Red";
//           } else if (match.color == kGreenTarget) {
//             colorString = "Green";
//           } else if (match.color == kYellowTarget) {
//             colorString = "Yellow";
//           } else {
//             colorString = "Unknown";
//           }
//         return colorString; 
//     }

//     public boolean isColor(String color) {
//        if(color == colorString) {
//             return true;
//        } else {
//            return false;
//        }  
//     }

//     public void spinSpinner(double speed) {
//         if(speed >= -1 && speed <= 1) {                
//             spinnerMotor.set(speed);
//         } else if(speed < -1) {
//             spinnerMotor.set(-1);
//         } else if(speed > 1) {
//             spinnerMotor.set(1);
//         } else {
//             spinnerMotor.set(0);
//         } 
//     }

//     public void stopSpinner() {
//         spinnerMotor.set(0);
//     }

// }