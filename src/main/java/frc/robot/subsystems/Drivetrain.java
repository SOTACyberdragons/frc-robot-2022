//                                                 @                             
//                                                  &@@                           
//                          * .                    * @@@                          
//                           * (@   ,                 @@@@                        
//                               @@@*       /          @@@@                       
//                                @@@@@@    @@(     ,* ,@@@@@                     
//                         %@@@@/*  @@@@@@@@       ,**. @@@@@@                    
//                      #********,    @@@@@@@@@@    ***  @@@@@@                   
//                   **********    /    @@@@@@@@@@@@   ,  @@@@@@                  
//                              &@@/  (@  (@@@@@@@@@@@@   @@@@@@@                 
//                            @@@@@//  @@@@@@@@@@@@@@@@@@& @@@@@@@                
//                          @@@@@@@//  @@@@@@@@# .@@@@@@@@@@@@@@@@                
//                         @@@@@@&///  %@@@@@@@@(  *  @@@@@@@@@@@@                
//                       *@@@@@//   @@@@@@@@@@@@@@%     @@@@@@@@@@@               
//                      .@@@@@@@@@@//   .@@@@@@@@@@@@@@  @@@@@@@@@@@              
//                      @@@@@@@@@@@@@@(/     @@@@@@@@@@@@@@@@@@@@@@@@@            
//                   @ %@@@@@@@@@@@@@@   ,  @@@@@@@@@@@@@@@@@@@@@@@@@@@           
//                  @@ @@@@@@@@@@@@@   .             *@@@@@@@@@  @@@@@@#          
//                 @@@ @@@@@@@@@@@@%   *******@@@&///     &@@@@@@@@@@@@@          
//                 @**  @@@@@@@@@@@   ******@@@@@@,          @@@@@@@@@@           
//                 #*** @@@@@@@@@@@   *****@@@@@                  @@@@*           
//                ***   @@@@@@@@@@@  ,****@@@,                                    
//                 *      @@@@@@@@@@.  *****@@                                    
//                          @@@@@@@@@#   ***%@                                    
//                           ,@@@@@@@@@    ***@,  /                               
//                              @@@@@@@@@(    ***   //////*.     */               
//                                 //@@@@@@%/      *    ///////                   
//                                 @    //////////                                
//                                   @@**                                         
//                                       @*****                                   
//                                             *                                  

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.TalonFXConfig;

public class Drivetrain extends SubsystemBase {

    // Create Falcon 500 motor objects
    public WPI_TalonFX leftSlave, leftMaster, rightSlave, rightMaster;
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    TalonFXInvertType leftInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"
	TalonFXInvertType rightInvert = TalonFXInvertType.CounterClockwise; // Same as invert = "false"
    ; //Same as invert = "true"


    // Falcon 500 faults objects
    private Faults faults = new Faults();

    // Instantiate the gyro
    public final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(RobotMap.PIGEON_IMU);
    public final boolean gyroReversed = false; // Was true

    // Create drive object
    private final DifferentialDrive m_drive;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    // Wheel Speed containers
    public static double m_leftWheelSpeed;
    public static double m_rightWheelSpeed;

    //tracking variables 
    boolean _firstCall = false; 
    boolean _state = false; 
    double _targetAngle = 0;

    // Field2d object for Smartdashboard display
    // private Field2d m_field = new Field2d();

    public Drivetrain() {

        leftMaster = TalonFXConfig.generateDefaultTalon(RobotMap.LEFT_MASTER);
        leftSlave = TalonFXConfig.generateDefaultTalon(RobotMap.LEFT_SLAVE);
        rightMaster = TalonFXConfig.generateDefaultTalon(RobotMap.RIGHT_MASTER);
        rightSlave = TalonFXConfig.generateDefaultTalon(RobotMap.RIGHT_SLAVE);

        // Factory default all hardware to prevent unexpected behavior
        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();
        m_gyro.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setSensorPhase(false);
        leftMaster.setSensorPhase(false);

        rightMaster.setInverted(rightInvert);
        rightSlave.setInverted(rightInvert);
        leftMaster.setInverted(leftInvert);
        leftSlave.setInverted(leftInvert);

        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave.setNeutralMode(NeutralMode.Coast);

        m_odometry = new DifferentialDriveOdometry(getHeading());
        m_drive = new DifferentialDrive(leftMaster, rightMaster);

        initDriveForPID();

    

        // Populate field position in Smartdashboard
        // SmartDashboard.putData("Field", m_field);
    }

    /**
     * Copy of example code from this example: https://github.com/Team2168/2020_Main_Robot/blob/master/src/main/java/org/team2168/subsystems/Drivetrain.java
     */
    private void initDriveForPID() {

        /**
         * NOTE: Do not get confused when they reference talons, I know there 
         * are falcons on the drivetrain, those are still talon FX
         */

    
	/** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source

    /* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
    rightConfig.remoteFilter0.remoteSensorDeviceID = leftMaster.getDeviceID(); //Device ID of Remote Source
    rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
    
    /* Now that the Left sensor can be used by the master Talon,
      * set up the Left (Aux) and Right (Master) distance into a single
      * Robot distance as the Master's Selected Sensor 0. */
    setRobotDistanceConfigs(rightInvert, rightConfig);

    /* FPID for Distance */
    rightConfig.slot0.kF = Constants.kGains_Distance.kF;
    rightConfig.slot0.kP = Constants.kGains_Distance.kP;
    rightConfig.slot0.kI = Constants.kGains_Distance.kI;
    rightConfig.slot0.kD = Constants.kGains_Distance.kD;
    rightConfig.slot0.integralZone = Constants.kGains_Distance.kIzone;
    rightConfig.slot0.closedLoopPeakOutput = Constants.kGains_Distance.kPeakOutput;

    /** Heading Configs */
    rightConfig.remoteFilter1.remoteSensorDeviceID = m_gyro.getDeviceID();    //Pigeon Device ID
    rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
    rightConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1; //Set as the Aux Sensor
    rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.kPigeonUnitsPerRotation; //Convert Yaw to tenths of a degree

    /* false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
      *   This is typical when the master is the right Talon FX and using Pigeon
      * 
      * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
      *   This is typical when the master is the left Talon FX and using Pigeon
      */
    rightConfig.auxPIDPolarity = false;

    /* FPID for Heading */
    rightConfig.slot1.kF = Constants.kGains_Turning_Straight.kF;
    rightConfig.slot1.kP = Constants.kGains_Turning_Straight.kP;
    rightConfig.slot1.kI = Constants.kGains_Turning_Straight.kI;
    rightConfig.slot1.kD = Constants.kGains_Turning_Straight.kD;
    rightConfig.slot1.integralZone = Constants.kGains_Turning_Straight.kIzone;
    rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning_Straight.kPeakOutput;


    /* Config the neutral deadband. */
    leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    rightConfig.neutralDeadband = Constants.kNeutralDeadband;

    // _leftConfig.nominalOutputForward = 0.045; //0.08 
    // _leftConfig.nominalOutputReverse = -0.045;
    // _leftConfig.peakOutputForward = 1.0;
    // _leftConfig.peakOutputReverse = -1.0;
    // _rightConfig.nominalOutputForward = 0.045;
    // _rightConfig.nominalOutputReverse = -0.045;
    // _rightConfig.peakOutputForward = 1.0;
    // _rightConfig.peakOutputReverse = -1.0;


    /**
     * 1ms per loop.  PID loop can be slowed down if need be.
     * For example,
     * - if sensor updates are too slow
     * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
     * - sensor movement is very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    rightMaster.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
    rightMaster.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

    /* Motion Magic Configs */
    // _rightConfig.motionAcceleration = (int) (inches_per_sec_to_ticks_per_100ms(5.0*12.0)); //(distance units per 100 ms) per second //7500
    // _rightConfig.motionCruiseVelocity = (int) (inches_per_sec_to_ticks_per_100ms(10.0*12.0)); //distance units per 100 ms //10000


    /* APPLY the config settings */
    leftMaster.configAllSettings(leftConfig);
    rightMaster.configAllSettings(rightConfig);

    rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
    rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);


    /* Set status frame periods to ensure we don't have stale data */
    /* These aren't configs (they're not persistant) so we can set these after the configs.  */
    rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, Constants.kTimeoutMs);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
    m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, Constants.kTimeoutMs);

    } 

    /** 
   * Determines if SensorSum or SensorDiff should be used 
   * for combining left/right sensors into Robot Distance.  
   * 
   * Assumes Aux Position is set as Remote Sensor 0.  
   * 
   * configAllSettings must still be called on the master config
   * after this function modifies the config values. 
   * https://github.com/Team2168/2020_Main_Robot/blob/master/src/main/java/org/team2168/subsystems/Drivetrain.java
   * 
   * @param masterInvertType Invert of the Master Talon
   * @param masterConfig Configuration object to fill
   */
  private void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
    /**
     * Determine if we need a Sum or Difference.
     * 
     * The auxiliary Talon FX will always be positive
     * in the forward direction because it's a selected sensor
     * over the CAN bus.
     * 
     * The master's native integrated sensor may not always be positive when forward because
     * sensor phase is only applied to *Selected Sensors*, not native
     * sensor sources.  And we need the native to be combined with the 
     * aux (other side's) distance into a single robot distance.
     */

    /* THIS FUNCTION should not need to be modified. 
        This setup will work regardless of whether the master
        is on the Right or Left side since it only deals with
        distance magnitude.  */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.Clockwise){
      /* 
        If master is inverted, that means the integrated sensor
        will be negative in the forward direction.
        If master is inverted, the final sum/diff result will also be inverted.
        This is how Talon FX corrects the sensor phase when inverting 
        the motor direction.  This inversion applies to the *Selected Sensor*,
        not the native value.
        Will a sensor sum or difference give us a positive total magnitude?
        Remember the Master is one side of your drivetrain distance and 
        Auxiliary is the other side's distance.
          Phase | Term 0   |   Term 1  | Result
        Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
        Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
        Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
      */

      masterConfig.diff0Term = FeedbackDevice.IntegratedSensor; //Local Integrated Sensor
      masterConfig.diff1Term = FeedbackDevice.RemoteSensor0;   //Aux Selected Sensor
      masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorDifference; //Diff0 - Diff1
    } else {
      /* Master is not inverted, both sides are positive so we can sum them. */
      masterConfig.sum0Term = FeedbackDevice.RemoteSensor0;    //Aux Selected Sensor
      masterConfig.sum1Term = FeedbackDevice.IntegratedSensor; //Local IntegratedSensor
      masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum; //Sum0 + Sum1
    }

    /* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
        the real-world value */
    masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
    }


    /** 
     * Zeros encoders and gyro
     */
    public void zeroSensors() {
		leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		m_gyro.setYaw(0, Constants.kTimeoutMs);
		m_gyro.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Integrated Encoders + Pigeon] All sensors are zeroed.\n");
	}

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(getHeading(), getLeftDistance(), getRightDistance());

        // Smart Dashboard display
        // SmartDashboard.putNumber("Robot X", getPose().getTranslation().getX());
        // SmartDashboard.putNumber("Robot Y", getPose().getTranslation().getY());
        // SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
        // SmartDashboard.putNumber("Robot Rotation", getRotation());
        // SmartDashboard.putNumber("Drive Distance: ", getAverageDistance());
        // SmartDashboard.putNumber("Left Encoder: ", getLeftEncoder());
        // SmartDashboard.putNumber("Right Encoder: ", getRightEncoder());
        // SmartDashboard.putNumber("Turn Rate: ", getTurnRate());

        // Update field position
        // m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftMaster.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse,
                rightMaster.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse);
    }

    public double getWheelRPM() {
        return (60 / ((2 * Math.PI) * Constants.kWheelRadiusMeters))
                * (leftMaster.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getHeading());
    }

    public void resetOdometry() {
        resetOdometry(getPose());
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

    /**
     * This method checks if the robot's left encoder is out of phase
     * 
     * @return returns true if it is
     * 
     */
    public boolean leftEncoderOutOfPhase() {
        return faults.SensorOutOfPhase;
    }

    /**
     * Get the distance of the left wheels in meters (@TODO or inches?)
     * 
     * @return returns distance in meters
     */
    public double getLeftDistance() {
        return getLeftEncoder() * Constants.kEncoderDistancePerPulse;
    }

    /**
     * Gets the distance of the right wheels in meters (@TODO or inches?)
     * 
     * @return returns the distance in meters.
     * 
     */
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
     * Gets the heading of the robot in UNIT, X is positive and Y is negative. 
     * 
     * @return angle
     */
    public Rotation2d getHeading() {
        final PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        double angle = m_gyro.getFusedHeading(fusionStatus);
        double newAngle = Math.IEEEremainder(angle, 360) * (gyroReversed ? -1.0 : 1.0);
        return Rotation2d.fromDegrees(newAngle);
    }

    /**
     * Gets healthing of the robot in UNIT, out of 360. X is positive and Y is negative. 
     * 
     * @return angle
     */
    public double getHeadingDouble() {
        final PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        double angle = m_gyro.getFusedHeading(fusionStatus);
        return Math.IEEEremainder(angle, 360) * (gyroReversed ? -1.0 : 1.0);
    }

    /**
     * 
     * 
     * @return
     */
	public double getAngle() {
		final PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		final double[] xyz_dps = new double[3];
		m_gyro.getRawGyro(xyz_dps);
		final double currentAngle = m_gyro.getFusedHeading(fusionStatus);
		return currentAngle;
	}

    /**
     * 
     * @return
     */
    public double getRotation() {
        final PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        return m_gyro.getFusedHeading(fusionStatus);
    }

    /**
     * 
     * @param angle
     */
	public void setAngle(final double angle) {
		final double distance = (getLeftEncoder() + getRightEncoder()) / 2;
		final double totalAngle = angle + getAngle();
		// leftMaster.set(ControlMode.PercentOutput, distance, DemandType.ArbitraryFeedForward, totalAngle);
		rightMaster.set(ControlMode.PercentOutput, distance, DemandType.ArbitraryFeedForward, -totalAngle);
	}

    public void setAnglePID(double angle) { 
        double target_sensorUnits = 2 * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;

        rightMaster.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, angle);
        leftMaster.follow(rightMaster, FollowerType.AuxOutput1);
    }
    /**
     * 
     * @param distance
     */
    public void setDistance(final double distance) {
        final double distanceTicks = distance / Constants.kEncoderDistancePerPulse;
        final double totalDistance = ((getLeftEncoder() + getRightEncoder()) / 2) + distanceTicks;
        final double angle = getAngle();
        rightMaster.set(ControlMode.MotionMagic, totalDistance, DemandType.AuxPID, angle);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        double[] xyz_dps = new double[3];
        m_gyro.getRawGyro(xyz_dps);
        double currentAngularRate = xyz_dps[2];
        return currentAngularRate;
    }

    /**
     * 
     * @param xSpeed
     * @param zRotation
     * 
     */
    public void m_drive(final double xSpeed, final double zRotation) {
        m_drive.arcadeDrive(xSpeed, zRotation, true);
    }
}
