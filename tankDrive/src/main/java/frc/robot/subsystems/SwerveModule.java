package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    private Vector2d m_velocityVector;

    private CANSparkMax m_driveMotor, m_rotionMotor;
    private double m_targetAngle;

    public SwerveModule(int driveId, int rotionId){
        m_driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        m_rotionMotor = new CANSparkMax(rotionId, MotorType.kBrushless);
    }


    public void setState(Vector2d state){
        double flipped = flipedPath(m_velocityVector.theta(), state.theta()),
         shortestPath = shortestPath(m_velocityVector.theta(), state.theta());
        if(flipped > shortestPath){
            m_targetAngle = shortestPath(m_velocityVector.theta(), state.theta());
            m_driveMotor.getPIDController().setReference(state.mag(), ControlType.kVelocity);
        }
        else{
            m_targetAngle = flipedPath(m_velocityVector.theta(), state.theta());
            m_driveMotor.getPIDController().setReference(-state.mag(), ControlType.kVelocity);
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
        double angle = m_rotionMotor.getEncoder().getPosition();
        if(MathUtil.isNear(m_targetAngle, angle, SwerveConsts.MODULE_ROTATION_TOLORENCE)){
            m_rotionMotor.getPIDController().setReference(m_targetAngle, ControlType.kPosition);
        }
       
    }

    
}
