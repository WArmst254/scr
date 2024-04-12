package frc2024.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.team4522.lib.config.DeviceConfig;
import com.team4522.lib.util.OrchestraUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.RobotContainer;
import frc2024.Constants.AmperConstants;
import frc2024.Constants.SuperstructureState;
import frc2024.Constants.Ports;
import frc2024.Constants.RobotMode;

public class Amper extends SubsystemBase{
        
    private TalonFX m_amperMotor;
    private TimeOfFlight m_beam;

    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
    private Debouncer m_beamDebouncer = new Debouncer(0.14, DebounceType.kBoth);

    public Amper(){
        m_amperMotor = new TalonFX(Ports.CONVEYOR_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        m_beam = new TimeOfFlight(Ports.CONVEYOR_BEAM_ID);

        configShooterMotors();
        
        OrchestraUtil.add(m_amperMotor);
    }
    
    private void configShooterMotors() {
        DeviceConfig.configureTalonFX("Amper Motor", m_amperMotor, DeviceConfig.conveyorFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        ParentDevice.optimizeBusUtilizationForAll(m_amperMotor);
    }
    
    public void setNeutralMode(NeutralModeValue mode){
        m_amperMotor.setNeutralMode(mode);
    }

    public void setAmper(ControlRequest control){
        m_amperMotor.setControl(control);
    }

    public void setAmperOutput(double output){
        setAmper(m_dutyCycleRequest.withOutput(output));
    }

    public double getRPM(){
        return m_amperMotor.getVelocity().getValueAsDouble()*60;
    }
    
    public BooleanSupplier hasPiece(boolean trapBeam){
        return trapBeam ? () -> m_beamDebouncer.calculate((m_beam.getRange() < AmperConstants.BEAM_THRESHOLD)) : () -> (m_beam.getRange() < AmperConstants.BEAM_THRESHOLD);
    }

    public void stop(){
        m_amperMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // System.out.println(!m_beam.get());
        if(Constants.MODE == RobotMode.COMP){
            logOutputs();
        }
    }

    public void logOutputs(){
        ScreamUtil.logBasicMotorOutputs("Amper", m_amperMotor);
    }

    public Command dutyCycleCommand(double output){
        return run(() -> setAmperOutput(output));
    }

    public Command stopCommand(){
        return Commands.runOnce(() -> stop(), this);
    }

    public Command scoreCommand(){
        return new ConditionalCommand(
            dutyCycleCommand(AmperConstants.AMP_OUTPUT), 
            new ConditionalCommand(
                dutyCycleCommand(AmperConstants.SHOOT_OUTPUT), 
                stopCommand(), 
                () -> RobotContainer.getCurrentState().get() != SuperstructureState.HOME
                && RobotContainer.getCurrentState().get() != SuperstructureState.HOME_ENDGAME
                && RobotContainer.getCurrentState().get() != SuperstructureState.NONE), 
            () -> RobotContainer.getCurrentState().get() == SuperstructureState.AMP);
    }
}
