package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Utils.Vector2d;

public class SwerveModule extends SubsystemBase {

    private Vector2d m_velocityVector;

    private CANSparkMax m_driveMotor, m_rotionMotor;
    private double m_targetAngle;

    public SwerveModule(int driveId, int rotionId){
        m_driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        m_rotionMotor = new CANSparkMax(rotionId, MotorType.kBrushless);
    }

    public void setState(Vector2d state){
        m_velocityVector = state;
        m_driveMotor.getPIDController().setReference(state.mag(), ControlType.kVelocity);
        turnTo(state.theta());
    }

    public Vector2d getstate(){
        return m_velocityVector;
    }

    private double shortestPath(double current, double target){
        double deltaAngle = (target % 360) - (current % 360);
        double flipDeltaAngle = (360 - deltaAngle) % 360; 
        if(Math.abs(deltaAngle) > 180){
            return flipDeltaAngle;
            // deltaAngle = -1.0 * Math.signum(deltaAngle) * 360.0 + deltaAngle;
        }
        return deltaAngle;
    }

    public void turnTo(double targetAngle){
        m_targetAngle = shortestPath(m_velocityVector.theta(), targetAngle);
    }

    @Override
    public void periodic(){
        if(MathUtil.isNear(m_targetAngle, m_rotionMotor.getEncoder().getPosition(), SwerveConsts.MODULE_ROTATION_TOLORENCE)){
            m_rotionMotor.getPIDController().setReference(m_targetAngle, ControlType.kPosition);
        }
       
    }

    
}
