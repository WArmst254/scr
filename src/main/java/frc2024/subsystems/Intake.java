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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.Constants;
import frc2024.Constants.IntakeConstants;
import frc2024.Constants.Ports;
import frc2024.Constants.RobotMode;

public class Intake extends SubsystemBase{
    private TalonFX m_intakeMotor;
    private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

     private TimeOfFlight m_beam;
     private Debouncer m_beamDebouncer = new Debouncer(0.14, DebounceType.kBoth);

    public Intake(){
        m_intakeMotor = new TalonFX(Ports.RIGHT_INTAKE_MOTOR_ID, Ports.RIO_CANBUS_NAME);
        configIntakeMotors();

        OrchestraUtil.add(m_intakeMotor);
    }

    public void configIntakeMotors(){
        DeviceConfig.configureTalonFX("Intake Motor", m_intakeMotor, DeviceConfig.intakeFXConfig(), Constants.DEVICE_LOOP_TIME_HZ);
        ParentDevice.optimizeBusUtilizationForAll(m_intakeMotor);
    }

    public void setNeutralMode(NeutralModeValue mode){
        m_intakeMotor.setNeutralMode(mode);
    }

    public void setIntake(ControlRequest control){
        m_intakeMotor.setControl(control);
    }

    public void setIntakeOutput(double output){
        setIntake(m_dutyCycleRequest.withOutput(output));
    }

    public double getRPM(){
        return m_intakeMotor.getVelocity().getValueAsDouble()*60.0;
    }

    public BooleanSupplier hasPiece(boolean trapBeam){
        return trapBeam ? () -> m_beamDebouncer.calculate((m_beam.getRange() < IntakeConstants.BEAM_THRESHOLD)) : () -> (m_beam.getRange() < IntakeConstants.BEAM_THRESHOLD);
    }

    public void stop(){
        m_intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if(Constants.MODE == RobotMode.COMP){
            logOutputs();
        }
    }

    public void logOutputs(){
        ScreamUtil.logBasicMotorOutputs("Intake", m_intakeMotor);
    }

    public Command dutyCycleCommand(double output){
        return run(() -> setIntakeOutput(output));
    }

    public Command stopCommand(){
        return Commands.runOnce(() -> stop(), this);
    }
}
