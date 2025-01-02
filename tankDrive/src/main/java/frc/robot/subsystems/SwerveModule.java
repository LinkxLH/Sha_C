package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;

public class SwerveModule extends SubsystemBase {

    private Vector2d m_velocityVector;

    private EverMotorController m_driveMotor, m_rotionMotor;
    private double m_targetAngle;
    private EverPIDController m_drivePid, m_rotionPid;
    private EverEncoder m_Encoder;

    public SwerveModule(int driveId, int rotionId){
        m_driveMotor = new EverSparkMax(0);
        m_rotionMotor = new EverSparkMax(0);
        m_drivePid = new EverSparkMaxPIDController((EverSparkMax)m_driveMotor);
        m_rotionPid = new EverSparkMaxPIDController((EverSparkMax)m_rotionMotor);
        m_Encoder = new EverSparkInternalEncoder((EverSparkMax)m_rotionMotor);
    }


    public void setState(Vector2d state){
        double flipped = flipedPath(m_velocityVector.theta(), state.theta()),
         shortestPath = shortestPath(m_velocityVector.theta(), state.theta());
        if(flipped > shortestPath){
            m_targetAngle = shortestPath(m_velocityVector.theta(), state.theta());
            m_drivePid.activate(state.mag(), ControlType.kVel);
        }
        else{
            m_targetAngle = flipedPath(m_velocityVector.theta(), state.theta());
            m_drivePid.activate(-state.mag(), ControlType.kVel);
        }
        m_velocityVector = state;
    }

    public Vector2d getstate(){
        return m_velocityVector;
    }

    private double shortestPath(double current, double target){
        double deltaAngle = (target % 360) - (current % 360);
        if(Math.abs(deltaAngle) > 180){
            deltaAngle = -1.0 * Math.signum(deltaAngle) * 360.0 + deltaAngle;
        }
        return deltaAngle;
    }

    private double flipedPath(double current, double target){
        double flipDeltaAngle = (360 - current) % 360; 
        return flipDeltaAngle;
    }

    @Override
    public void periodic(){
        double angle = m_Encoder.getPos();
        if(MathUtil.isNear(m_targetAngle, angle, SwerveConsts.MODULE_ROTATION_TOLORENCE)){
            m_rotionPid.activate(m_targetAngle, ControlType.kPos);
        }
       
    }

    
}
