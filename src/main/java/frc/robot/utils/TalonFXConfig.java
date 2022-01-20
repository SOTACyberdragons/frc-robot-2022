package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
 /**
  * BaseFXConfig Class
  * 
  * <p>All settings are from http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_f_x_configuration.html </p>
  */
public class TalonFXConfig extends TalonFXConfiguration{

    public static WPI_TalonFX configureTalon(WPI_TalonFX talon){
        talon.configAllSettings(new TalonFXConfig());
        return talon;
    }

    public static WPI_TalonFX generateDefaultTalon(int deviceID){
        return configureTalon(new WPI_TalonFX(deviceID));
    }

    public TalonFXConfig(){
        this.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        this.initializationStrategy = SensorInitializationStrategy.BootToZero;
        this.integratedSensorOffsetDegrees = 0;
        this.motorCommutation = MotorCommutation.Trapezoidal;
        this.clearPositionOnLimitF = false;
        this.clearPositionOnLimitR = false;
        this.closedloopRamp = 0.08;
        this.enableOptimizations = true;
        this.forwardLimitSwitchDeviceID = 0;
        this.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
        this.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
        this.forwardSoftLimitEnable = false;
        this.motionAcceleration = 0;
        this.motionCruiseVelocity = 0;
        this.motionCurveStrength = 0;
        this.nominalOutputForward = 0.0;
        this.nominalOutputReverse = 0.0;
        this.openloopRamp = 0.08;
        this.peakOutputForward = 1.0;
        this.peakOutputReverse = 1.0;
        this.remoteSensorClosedLoopDisableNeutralOnLOS = false;
        this.reverseLimitSwitchDeviceID = 0;
        this.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
        this.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
        this.reverseSoftLimitEnable = false;
        this.softLimitDisableNeutralOnLOS = false;

        }  
    }