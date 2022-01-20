package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Instrum {

	private static int _loops = 0;
	private static int _timesInMotionMagic = 0;

	public static void Process(TalonSRX tal, StringBuilder sb)
	{
		/* smart dash plots */
		SmartDashboard.putNumber("SensorVel", tal.getSelectedSensorVelocity(Constants.PID_LOOP_IDX));
		SmartDashboard.putNumber("SensorPos", tal.getSelectedSensorPosition(Constants.PID_LOOP_IDX));
		SmartDashboard.putNumber("MotorOutputPercent", tal.getMotorOutputPercent());
		SmartDashboard.putNumber("ClosedLoopError", tal.getClosedLoopError(Constants.PID_LOOP_IDX));

		/* check if we are motion-magic-ing */
		if (tal.getControlMode() == ControlMode.MotionMagic) {
			++_timesInMotionMagic;
		} else {
			_timesInMotionMagic = 0;
		}
		if (_timesInMotionMagic > 10) {
			/* print the Active Trajectory Point Motion Magic is servoing towards */
			SmartDashboard.putNumber("ClosedLoopTarget", tal.getClosedLoopTarget(Constants.PID_LOOP_IDX));
			SmartDashboard.putNumber("ActTrajVelocity", tal.getActiveTrajectoryVelocity());
			SmartDashboard.putNumber("ActTrajPosition", tal.getActiveTrajectoryPosition());
			SmartDashboard.putNumber("ActTrajHeading", tal.getActiveTrajectoryPosition(1));
		}
		/* periodically print to console */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(sb.toString());
		}
		/* clear line cache */
		sb.setLength(0);
	}
}