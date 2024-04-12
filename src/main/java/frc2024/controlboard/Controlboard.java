package frc2024.controlboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team4522.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A utility class that contains button bindings.
 * 
 * Controlboard allows easy reference of custom button associations.
 */
public class Controlboard{

    public static final double STICK_DEADBAND = 0.05;
    public static final double TRIGGER_DEADBAND = 0.10;
    public static final Rotation2d SNAP_TO_POLE_THRESHOLD = Rotation2d.fromDegrees(7.0);

    public static final CommandXboxController driverController_Command = new CommandXboxController(0);
    public static final CommandXboxController operatorController_Command = new CommandXboxController(1);

    public static boolean fieldCentric = true;

    public static Command driverRumbleCommand(RumbleType type, double value, double time){
        return Commands.startEnd(
            () -> driverController_Command.getHID().setRumble(type, value),
            () -> driverController_Command.getHID().setRumble(type, 0.0))
            .withTimeout(time);
    }

    public static Command operatorRumbleCommand(RumbleType type, double value, double time){
        return Commands.startEnd(
            () -> operatorController_Command.getHID().setRumble(type, value),
            () -> operatorController_Command.getHID().setRumble(type, 0.0))
            .withTimeout(time);
    }

    /**
     * Retrieves the swerve translation from the driver controller.
     * 
     * @return A DoubleSupplier array representing the x and y values from the controller.
     */
    public static DoubleSupplier[] getTranslation(){
        return new DoubleSupplier[]{
            () -> -MathUtil.applyDeadband(driverController_Command.getLeftY(), STICK_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController_Command.getLeftX(), STICK_DEADBAND),
        };
    }


    public static DoubleSupplier getSnapAngle(){
        return () -> {
            if(driverController_Command.getHID().getAButton()){
                return 90.0;
            } else if(driverController_Command.getHID().getYButton()){
                return AllianceFlipUtil.Number(0.0, 180.0);
            } else {
                return -1.0;
            }
        };
    }

    public static Trigger snapAnglePresent(){
        return new Trigger(() -> getSnapAngle().getAsDouble() != -1.0);
    }

    public static BooleanSupplier getSlowMode(){
        return () -> driverController_Command.getHID().getLeftTriggerAxis() >= TRIGGER_DEADBAND;
    }

    /**
     * Retrieves the rotation value from the driver controller.
     *
     * @return A DoubleSupplier representing the rotation.
     */
    public static DoubleSupplier getRotation() {
        return () -> -MathUtil.applyDeadband(driverController_Command.getRightX(), STICK_DEADBAND);
    }

    public static Trigger dodgeRight(){
        return driverController_Command.rightStick();
    }

    public static Trigger dodgeLeft(){
        return driverController_Command.leftStick();
    }

    /**
     * Retrieves whether to zero the gyro from the driver controller.
     *
     * @return A Trigger representing the state of the start button.
     */
    public static Trigger zeroGyro() {
        return driverController_Command.back();
    }

    public static Trigger resetPose() {
        return driverController_Command.povUp();
    }

    public static Trigger resetPose_Apriltag() {
        return operatorController_Command.povUp();
    }

    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public static BooleanSupplier getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        driverController_Command.start().onTrue(Commands.runOnce(() -> fieldCentric =! fieldCentric));
        return () -> fieldCentric;
    }

    public static Trigger manualMode(){
    /* Changes how the angle/height of the pivot and elevator are input */
        return new Trigger(operatorController_Command.povRight());
    }

    /* Automation */
    public static final Trigger prepShot(){
        /* Uses a toggle switch to enable or disable automatic prep shooting */
        return new Trigger(operatorController_Command.back());
    }

    public static final Trigger autoFire(){
        /* Uses a toggle switch to enable or disable automatic firing when requirements are met */
        /* return new Trigger(() -> buttonBoard.getRawSwitch(4)); */
        return driverController_Command.leftBumper();
        //return new Trigger(() -> buttonBoard.getRawButton(3));
    }


    public static final Trigger virtualAutoFire(){
        return new Trigger(operatorController_Command.povRight());
    }

    public static final BooleanSupplier endGameMode(){
        return () -> operatorController_Command.getRightTriggerAxis() > 0.1;
    }

    /* public static final Trigger rehome(){
        return new Trigger(() -> buttonBoard.getRawButton(3));
    } */

    public static final Trigger snapToSpeaker(){
        return driverController_Command.a();
    }

    public static final Trigger shooterIntoConveyor(){
        return operatorController_Command.leftBumper();
    }

    public static final Trigger eject(){
        return driverController_Command.povRight();
    }


    /* Pivot */
    public static final DoubleSupplier getManualPivotOutput(){
        return () -> -MathUtil.applyDeadband(operatorController_Command.getRightY(), STICK_DEADBAND)/2;
    }

    public static final BooleanSupplier increasePivot(){
        return () -> operatorController_Command.getHID().getPOV() == 0.0;
    }

    public static final BooleanSupplier decreasePivot(){
        return () -> operatorController_Command.getHID().getPOV() == 180.0;
    }

    /* Elevator */
    public static final DoubleSupplier getManualElevatorOutput(boolean driverController){
        return () -> driverController
               ? (-MathUtil.applyDeadband(driverController_Command.getRightY(), STICK_DEADBAND))*12.0
               : (-MathUtil.applyDeadband(operatorController_Command.getLeftY(), STICK_DEADBAND))*6.0;
    }

    public static final Trigger elevatorDown_MAX(){
        return operatorController_Command.a();
    }  

    /* Intake */
    public static final Trigger intakeFromFloor(){
        return driverController_Command.rightTrigger(TRIGGER_DEADBAND).and(new Trigger(endGameMode()).negate());
    }

    public static final Trigger score(){
        return driverController_Command.rightBumper();
    }

    public static final Trigger autoPickupFromFloor(){
        return driverController_Command.leftBumper();
    }
}
