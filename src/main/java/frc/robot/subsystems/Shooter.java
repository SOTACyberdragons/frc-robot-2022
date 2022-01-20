// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.RobotMap;
// import frc.robot.oi.limelightvision.limelight.LimeLight;
// import frc.robot.utils.TalonFXConfig;

// public class Shooter extends SubsystemBase {

//     private WPI_TalonFX leftMotor , rightMotor; 
//     private double speed = -0.75;
//     private double goalHeight = 7*12;
//     private double cameraHeight = 28;
//     private double cameraAngle = 131.58-90;

//     private double wheelRadius = 8;
//     private double distancePerPulse = wheelRadius * Math.PI / 2048;
//     private double kP = 0.1;
//     private double kI = 0.001;
//     private double kD = 0;
//     private double kF = 0;


//     LimeLight limeLight = new LimeLight();
//     public Shooter() {
//         rightMotor = TalonFXConfig.generateDefaultTalon(RobotMap.LEFT_SHOOTER_MOTOR);
//         rightMotor.configFactoryDefault();
//         rightMotor.setInverted(true);

//         leftMotor = TalonFXConfig.generateDefaultTalon(RobotMap.RIGHT_SHOOTER_MOTOR);
//         leftMotor.configFactoryDefault();
//         leftMotor.setInverted(false);

//         rightMotor.follow(leftMotor);

//         leftMotor.configFactoryDefault();
//         rightMotor.configFactoryDefault();

//         rightMotor.setNeutralMode(NeutralMode.Coast);
//         leftMotor.setNeutralMode(NeutralMode.Coast);

//         leftMotor.configNominalOutputForward(0, Constants.TIMEOUT_MS);
// 		leftMotor.configNominalOutputReverse(0, Constants.TIMEOUT_MS);
// 		leftMotor.configPeakOutputForward(1, Constants.TIMEOUT_MS);
// 		leftMotor.configPeakOutputReverse(-1, Constants.TIMEOUT_MS);

// 		/* Config the Velocity closed loop gains in slot0 */
// 		leftMotor.config_kF(0, kF, 0);
// 		leftMotor.config_kP(0, kP, 0);
// 		leftMotor.config_kI(0, kI, 0);
// 		leftMotor.config_kD(0, kD, 0);

//     }
    
//     public double getDistanceFromGoal() {
//         double verticalAngleToTarget = limeLight.getdegVerticalToTarget();
//         double distance = (goalHeight - cameraHeight) / Math.tan(verticalAngleToTarget + cameraAngle);
//         return distance;
//     }

//     public double getTargetVelocity() {
//         double x1 = 0;
//         double x2 = 0;
//         double y1 = 0;
//         double y2 = 0;
//         double gInches = 386.09;
//         double horizonalVelocity = Math.sqrt(((gInches/2) * (x1-x2)) / ((y1/x1) - (y2/x2)));
//         double verticalVelocity = horizonalVelocity*gInches/x1 - gInches*x1/2*horizonalVelocity;
//         return Math.atan(verticalVelocity/horizonalVelocity);
//     }
    
//     //inches per 100
//     public void setVelocity(double targetVelocity) {
//         double targetVelocitySensorUnites = targetVelocity /distancePerPulse;
//         leftMotor.set(TalonFXControlMode.Velocity, targetVelocitySensorUnites);
//     }

//     public double getVelocity() {
//         return leftMotor.getSelectedSensorVelocity();
//     }
//     public LimeLight getLimeLight() {
// 		return limeLight;
//     }
    
//     public void shootOut() {
//         leftMotor.set(speed);
//     }

//     public void stop() {
//         leftMotor.set(0);
//     }


// }