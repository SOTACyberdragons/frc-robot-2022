package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.DriveStraight20inches;
// import frc.robot.commands.FeedBall;
// import frc.robot.commands.MoveIntake;
// import frc.robot.commands.ShootOut;
// import frc.robot.commands.SpinHopper;
// //import frc.robot.commands.SpinToColor;
// import frc.robot.commands.SpinIntake;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

 

    // Setting squaredInput to true decreases the sensitivity for tankdrive at lower
    // speeds
    private boolean squaredInput = true;

    public Command moveToAngle20;

    public Joystick leftDriveStick = new Joystick(0); //main driver
    public Joystick rightDriveStick = new Joystick(1); //main driver
    public Joystick leftAuxStick = new Joystick(2); //co-driver
    public Joystick rightAuxStick = new Joystick(3); //co-driver

    private XboxController gameController = new XboxController(4);
    public double getLeftX(){
        return gameController.getLeftX();
    }
    public double getRightX(){
        return gameController.getRightX();
    }
    public double getR2(){
        return gameController.getRightTriggerAxis();
    }
    public double getL2(){
        return gameController.getLeftTriggerAxis();
    }

    

    public OI() {
        /*
         * Set buttons
         */

        //  JoystickButton spinToColor = new JoystickButton(rightDriveStick, ButtonMap.SPIN_TO_COLOR); 
        //  JoystickButton liftIntake = new JoystickButton(leftDriveStick, ButtonMap.LIFT_INTAKE); 
        //  JoystickButton lowerIntake = new JoystickButton(leftDriveStick, ButtonMap.LOWER_INTAKE); 
        //  JoystickButton spinIntake = new JoystickButton(rightAuxStick, ButtonMap.SPIN_INTAKE);
        //  JoystickButton shootOut = new JoystickButton(leftAuxStick, ButtonMap.SHOOT_OUT);
         //JoystickButton driveStraight20inches = new JoystickButton(rightDriveStick, ButtonMap.DRIVE_STRAIGHT);
        //  JoystickButton feedBall = new JoystickButton(leftAuxStick, ButtonMap.FEED_BALL);
        //  JoystickButton feedBallOut = new JoystickButton(leftAuxStick, ButtonMap.FEED_BALL_OUT);
        //  JoystickButton spinHopper = new JoystickButton(rightAuxStick, ButtonMap.SPIN_HOPPER);

        //  final DriveStraight20inches testCmd = new DriveStraight20inches(Robot.drivetrain);
       
        /*
         * Set command
         */
        //spinToColor.whenPressed(new SpinToColor(/*Robot.getGameData()*/"Red"));
        // liftIntake.whenPressed(new MoveIntake("up"));
        // lowerIntake.whenPressed(new MoveIntake("down"));
        // spinIntake.whenPressed(new SpinIntake(1.0));
        // shootOut.whileHeld(new ShootOut());
        // driveStraight20inches.whileHeld(testCmd);
        // feedBall.whileHeld(new FeedBall(Robot.feeder, "in"));
        // feedBallOut.whileHeld(new FeedBall(Robot.feeder, "out"));
        // spinHopper.whileHeld(new SpinHopper());
        
	}
	
	public Joystick getLeftStick() {
		return leftDriveStick;
	}

	public Joystick getRightStick() {
		return rightDriveStick;
	}

    public boolean getShootButtonState()
    {
        return leftDriveStick.getRawButtonPressed(3);
    }

    public Joystick getLeftAuxStick() {
		return leftAuxStick;
    }
    
    public Joystick getRightAuxStick() {
        return rightAuxStick;
    }

    public boolean getSquaredInput() {
        return squaredInput;
    }
}