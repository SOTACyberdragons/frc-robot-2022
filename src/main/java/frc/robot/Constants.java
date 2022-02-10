package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DigitalSource;

public class Constants {

	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int SLOT_IDX = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int PID_LOOP_IDX = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int TIMEOUT_MS = 10;

	/**
	 * Talon
	 */
	public static final int TALON_MAX_OUTPUT = 1023;
	public static final int VERSA_ENCODER_TPR = 4096;
	public static final int MAG_ENCODER_TPR = 4096;

	/**
	 * Roborio
	 */
	public static final double CYCLE_SEC = 0.020; // 20 milliseconds
	public static final String DATA_DIR = "/home/lvuser/data_capture/";
	public static final String PATHFINDER_DIR = "/home/lvuser/pathfinder/";

	public static enum StartPosition {
		LEFT, CENTER, RIGHT, UNKNOWN
	};

	public static enum AutoChoice {
		DO_NOT_MOVE
	};

	// values from profiler
	public static final double ksVolts = 0.43389; // 0.589151 initially
	public static final double kvVoltSecondsPerMeter = 5.9696; // 5.9696 initially
	public static final double kaVoltSecondsSquaredPerMeter = 0.40192; // 0.11108 initially
	public static final double kTrackwidthMeters = 0.64;
	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
			kTrackwidthMeters);

	// Ramsete constants - TUNE THESE IF THE ROBOTS MOVES TOO QUICKLY!
	public static final double kMaxSpeedMetersPerSecond = 0.93088; // initially 3
	public static final double kMaxAccelerationMetersPerSecondSquared = 0.37894; // initially 3

	public static final double kRamseteB = 2; // initially 2
	public static final double kRamseteZeta = 0.7;

	public static final double kDistancePerPulse = 2048;

	// XBox Controller Drivetrain Constants
	public static final double kMaxDriveSpeed = .7;
	public static final double kMaxTurnSpeed = .7;

	// Updated drivetrain constants
	public static final int kEncoderCPR = 2048;
	public static final double kWheelDiameterMeters = 0.152;
	public static final double kGearRation = 7.95;
	public static final double kEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR)
			/ kGearRation;

	public static final double kPDriveVel = 0.0039019;

	public static int kTimeoutMs = 30;
	public static int kPIDLoopIdx = 0;

}