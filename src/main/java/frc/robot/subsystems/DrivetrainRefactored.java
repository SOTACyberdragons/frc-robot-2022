package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.utils.TalonFXConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainRefactored extends SubsystemBase {
    
    public WPI_TalonFX leftSlave, leftMaster, rightSlave, rightMaster;
    public final boolean gyroReversed = false;
    
    // Falcon 500 faults objects
    private Faults faults = new Faults();
    
    // Instantiate the gyro
    public final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(RobotMap.PIGEON_IMU);
    
    // Create drive and odometry objects
    public DifferentialDriveOdometry m_odometry;
    public Field2d m_field = new Field2d();
    
    private final DifferentialDrive m_drive;
    
    public DrivetrainRefactored() {
        
        leftMaster = new WPI_TalonFX(RobotMap.LEFT_MASTER);
        leftSlave = new WPI_TalonFX(RobotMap.LEFT_SLAVE);
        rightMaster = new WPI_TalonFX(RobotMap.RIGHT_MASTER);
        rightSlave = new WPI_TalonFX(RobotMap.RIGHT_SLAVE);
        
        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();
        
        leftMaster.configClosedloopRamp(0.1);
        leftSlave.configClosedloopRamp(0.1);
        rightMaster.configClosedloopRamp(0.1);
        rightSlave.configClosedloopRamp(0.1);
        
        leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
        leftSlave.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
        rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
        leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
        leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
        
        leftMaster.configNeutralDeadband(0.001, Constants.kTimeoutMs);
        leftSlave.configNeutralDeadband(0.001, Constants.kTimeoutMs);
        rightMaster.configNeutralDeadband(0.001, Constants.kTimeoutMs);
        leftMaster.configNeutralDeadband(0.001, Constants.kTimeoutMs);
        
        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
        leftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        leftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
        rightSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        rightSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
        
        leftMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
        leftMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
        leftMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
        leftMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        leftSlave.configNominalOutputForward(0, Constants.kTimeoutMs);
        leftSlave.configNominalOutputReverse(0, Constants.kTimeoutMs);
        leftSlave.configPeakOutputForward(1, Constants.kTimeoutMs);
        leftSlave.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        rightMaster.configNominalOutputForward(0, Constants.kTimeoutMs);
        rightMaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
        rightMaster.configPeakOutputForward(1, Constants.kTimeoutMs);
        rightMaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        rightSlave.configNominalOutputForward(0, Constants.kTimeoutMs);
        rightSlave.configNominalOutputReverse(0, Constants.kTimeoutMs);
        rightSlave.configPeakOutputForward(1, Constants.kTimeoutMs);
        rightSlave.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        
        leftMaster.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        leftMaster.configMotionAcceleration(6000, Constants.kTimeoutMs);
        leftSlave.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        leftSlave.configMotionAcceleration(6000, Constants.kTimeoutMs);
        rightMaster.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        rightMaster.configMotionAcceleration(6000, Constants.kTimeoutMs);
        rightSlave.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        rightSlave.configMotionAcceleration(6000, Constants.kTimeoutMs);
        
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        leftMaster.setInverted(false);
        leftSlave.setInverted(false);
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);
        
        leftMaster.setSafetyEnabled(false);
        leftSlave.setSafetyEnabled(false);
        rightMaster.setSafetyEnabled(false);
        rightSlave.setSafetyEnabled(false);
        
        rightMaster.setSensorPhase(false);
        leftMaster.setSensorPhase(false);
        
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave.setNeutralMode(NeutralMode.Coast);
        
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
        m_drive = new DifferentialDrive(leftMaster, rightMaster);
        
        //SmartDashboard.putData("Field", m_field);
    }
    
    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        //m_odometry.update(getHeading(), getLeftDistance(), getRightDistance());
        
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(),
        getRightDistance());
        
        var translation = m_odometry.getPoseMeters().getTranslation();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());
    }
    
    /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    
    // TODO Math for getWheelSpeeds() is incorrect.
    
    /**
    * Returns the current wheel speeds of the robot.
    *
    * @return The current wheel speeds.
    */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
        leftMaster.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse,
        rightMaster.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse);
        
        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2, 0, 1);
        
        // return Constants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);
    }
    
    /**
    * Resets the odometry to the specified pose.
    *
    * @param pose The pose to which to set the odometry.
    */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }
    
    /**
    * Drives the robot using arcade controls.
    *
    * @param fwd the commanded forward movement
    * @param rot the commanded rotation
    */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }
    
    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        m_drive.feed();
    }
    
    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, Constants.PID_LOOP_IDX, Constants.TIMEOUT_MS);
        rightMaster.setSelectedSensorPosition(0, Constants.PID_LOOP_IDX, Constants.TIMEOUT_MS);
    }
    
    /**
    * Gets the left drive encoder.
    *
    * @return the left drive encoder
    */
    public double getLeftEncoder() {
        return leftMaster.getSelectedSensorPosition(0);
    }
    
    /**
    * Gets the right drive encoder.
    *
    * @return the right drive encoder
    */
    public double getRightEncoder() {
        return rightMaster.getSelectedSensorPosition(0);
    }
    
    public boolean leftEncoderOutOfPhase() {
        return faults.SensorOutOfPhase;
    }
    
    public double getLeftDistance() {
        return getLeftEncoder() * Constants.kEncoderDistancePerPulse;
    }
    
    public double getRightDistance() {
        return getRightEncoder() * Constants.kEncoderDistancePerPulse;
    }
    
    /**
    * Gets the average distance of the two encoders.
    *
    * @return the average of the two encoder readings
    */
    public double getAverageDistance() {
        return (getRightDistance() + getLeftDistance()) / 2;
    }
    
    public void setDistance(final double distanceIn) {
        final double distanceTicks = distanceIn / Constants.kEncoderDistancePerPulse;
        final double totalDistance = (getLeftEncoder() + getRightEncoder()) / 2 + distanceTicks;
        final double angle = getHeading();
        rightMaster.set(ControlMode.MotionMagic, totalDistance, DemandType.AuxPID, angle);
    }
    
    /**
    * Sets the max output of the drive. Useful for scaling the drive to drive more
    * slowly.
    *
    * @param maxOutput the maximum output to which the drive will be constrained
    */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
    
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }
    
    /**
    * Returns the heading of the robot.
    *
    * @return the robot's heading in degrees, from -180 to 180
    */
    public double getHeading() {
        double ypr[] = {0,0,0};
        m_gyro.getYawPitchRoll(ypr);
        return (Math.IEEEremainder(ypr[0], 360.0d) * -1.0d);
    }
    
    public double getAngle() {
        return m_gyro.getAngle();
    }
    
    /**
    * Returns the turn rate of the robot.
    *
    * @return The turn rate of the robot, in degrees per second
    */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }
    
    // TODO Unable to figure out how to expose m_drive properly. This is probably causing DiffertialDrive to be instantiated twice.
    public void m_drive(final double xSpeed, final double zRotation) {
        m_drive.arcadeDrive(xSpeed, zRotation, true);
    }
    
}
