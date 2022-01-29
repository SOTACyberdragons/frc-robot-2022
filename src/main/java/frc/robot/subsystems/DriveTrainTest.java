// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.RobotMap;
// import frc.robot.utils.DriveConstants;
// import frc.robot.utils.TalonFXConfig;

// public class DriveTrainTest extends SubsystemBase {

//     public final static double WHEELBASE_WIDTH = 24.25;
// 	public final static double WHEEL_DIAMETER = 6;
// 	public final static double PULSE_PER_REVOLUTION = 2048; // from http://www.ctr-electronics.com/talon-fx.html#product_tabs_tech_specs
// 	public final static double REDUCTION_TO_ENCODER_FAST = PULSE_PER_REVOLUTION * 7.95; //11:42 24:50
// 	public final static double REDUCTION_TO_ENCODER_SLOW = (2048 * 42*60)/(11*14); //11:42 14:60
// 	public final static double DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER) / REDUCTION_TO_ENCODER_FAST;
// 	public final static double DISTANCE_PER_PULSE_METERS = Units.inchesToMeters(DISTANCE_PER_PULSE);
// 	public final static double MEETERS_PER_SECOND = 7.95 * 2 * Math.PI * Units.inchesToMeters(3) / 60;
// 	public final static double MAX_SPEED = 110.0;
// 	public static final double MAX_ACCEL = 1.0 / 0.0254; //0.2g in in/s^2
// 	public static final double MAX_JERK = 20 / 0.0254; // 30 / 0.0254; //from example code in Pathfinder
// 	public final double encoderMaxSpeed = 33000;
// 	public final double distanceBetweenWheels = 24;

//   public static WPI_TalonFX leftSlave = new WPI_TalonFX(RobotMap.LEFT_SLAVE);
//   public static WPI_TalonFX leftMaster = new WPI_TalonFX(RobotMap.LEFT_MASTER);
//   public static WPI_TalonFX rightSlave = new WPI_TalonFX(RobotMap.RIGHT_SLAVE);
//   public static WPI_TalonFX rightMaster = new WPI_TalonFX(RobotMap.RIGHT_MASTER);

//   // The robot's drive
//   private final DifferentialDrive m_drive = new DifferentialDrive(leftSlave, leftMaster);

//   // The left-side drive encoder

//   // The gyro sensor
//   private final Gyro m_gyro = new ADXRS450_Gyro();

//   // Odometry class for tracking robot pose
//   private final DifferentialDriveOdometry m_odometry;

//   /** Creates a new DriveSubsystem. 
//  * @param Constants */
//   public DriveTrainTest(Object Constants) {
//     // We need to invert one side of the drivetrain so that positive voltages
//     // result in both sides moving forward. Depending on how your robot's
//     // gearbox is constructed, you might have to invert the left side instead.
//     leftMaster = TalonFXConfig.generateDefaultTalon(RobotMap.LEFT_MASTER);
//         leftSlave = TalonFXConfig.generateDefaultTalon(RobotMap.LEFT_SLAVE);

//         rightMaster = TalonFXConfig.generateDefaultTalon(RobotMap.RIGHT_MASTER);
//         rightSlave = TalonFXConfig.generateDefaultTalon(RobotMap.RIGHT_SLAVE);

//         leftMaster.configFactoryDefault();
//         rightMaster.configFactoryDefault();
//         leftSlave.configFactoryDefault();
// 		rightSlave.configFactoryDefault();
		
// 		leftSlave.follow(leftMaster);
//         rightSlave.follow(rightMaster);

//         rightMaster.setNeutralMode(NeutralMode.Coast);
//         leftMaster.setNeutralMode(NeutralMode.Coast);
//         rightSlave.setNeutralMode(NeutralMode.Coast);
//         leftSlave.setNeutralMode(NeutralMode.Coast);
    

//     resetEncoders();
//     m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
//   }

//   public double getLeftRawEncoderTicks() {
//     return leftMaster.getSelectedSensorPosition(0);
//   }

//   public double getRightRawEncoderTicks() {
//     return rightMaster.getSelectedSensorPosition(0);
//   }

//   public double getDistance() {
//     return ((getLeftRawEncoderTicks() + getRightRawEncoderTicks()) / 2) * Constants.kDistancePerPulse;
//   }

//   @Override
//   public void periodic() {
//     // Update the odometry in the periodic block
//     m_odometry.update(
//         m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
//   }

//   /**
//    * Returns the currently-estimated pose of the robot.
//    *
//    * @return The pose.
//    */
//   public Pose2d getPose() {
//     return m_odometry.getPoseMeters();
//   }

//   /**
//    * Returns the current wheel speeds of the robot.
//    *
//    * @return The current wheel speeds.
//    */
//   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
//     return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
//   }

//   /**
//    * Resets the odometry to the specified pose.
//    *
//    * @param pose The pose to which to set the odometry.
//    */
//   public void resetOdometry(Pose2d pose) {
//     resetEncoders();
//     m_odometry.resetPosition(pose, m_gyro.getRotation2d());
//   }

//   /**
//    * Drives the robot using arcade controls.
//    *
//    * @param fwd the commanded forward movement
//    * @param rot the commanded rotation
//    */
//   public void arcadeDrive(double fwd, double rot) {
//     m_drive.arcadeDrive(fwd, rot);
//   }

//   /**
//    * Controls the left and right sides of the drive directly with voltages.
//    *
//    * @param leftVolts the commanded left output
//    * @param rightVolts the commanded right output
//    */
//   public void tankDriveVolts(double leftVolts, double rightVolts) {
//     m_leftMotors.setVoltage(leftVolts);
//     m_rightMotors.setVoltage(rightVolts);
//     m_drive.feed();
//   }

//   /** Resets the drive encoders to currently read a position of 0. */
//   public void resetEncoders() {
//     m_leftEncoder.reset();
//     m_rightEncoder.reset();
//   }

//   /**
//    * Gets the average distance of the two encoders.
//    *
//    * @return the average of the two encoder readings
//    */
//   public double getAverageEncoderDistance() {
//     return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
//   }

//   /**
//    * Gets the left drive encoder.
//    *
//    * @return the left drive encoder
//    */
//   public Encoder getLeftEncoder() {
//     return m_leftEncoder;
//   }

//   /**
//    * Gets the right drive encoder.
//    *
//    * @return the right drive encoder
//    */
//   public Encoder getRightEncoder() {
//     return m_rightEncoder;
//   }

//   /**
//    * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
//    *
//    * @param maxOutput the maximum output to which the drive will be constrained
//    */
//   public void setMaxOutput(double maxOutput) {
//     m_drive.setMaxOutput(maxOutput);
//   }

//   /** Zeroes the heading of the robot. */
//   public void zeroHeading() {
//     m_gyro.reset();
//   }

//   /**
//    * Returns the heading of the robot.
//    *
//    * @return the robot's heading in degrees, from -180 to 180
//    */
//   public double getHeading() {
//     return m_gyro.getRotation2d().getDegrees();
//   }

//   /**
//    * Returns the turn rate of the robot.
//    *
//    * @return The turn rate of the robot, in degrees per second
//    */
//   public double getTurnRate() {
//     return -m_gyro.getRate();
//   }
// }