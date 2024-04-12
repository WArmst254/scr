
package frc2024.dashboard.tabs;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.util.RunOnce;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.dashboard.ShuffleboardTabBase;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.swerve.Swerve;

public class MatchTab extends ShuffleboardTabBase {
    
    private static SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
    private Swerve swerve;
    private Elevator elevator;
    private Pivot pivot;

    public MatchTab(Swerve swerve, Elevator elevator, Pivot pivot) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.pivot = pivot;

        checkNulls(this.swerve, this.elevator, this.pivot);
    }

    private Field2d m_field2d = new Field2d();
 
    
    private static GenericEntry m_coast;
    private static boolean m_lastNeutralMode;
    private static RunOnce m_neutralModeChanger = new RunOnce();

    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Match");

       // m_autoChooserEntry = createSendableEntry("Auto Chooser", m_autoChooser, new EntryProperties(0, 0, 4, 2));
        //m_matchTimeEntry = createNumberEntry("Match Time", 0.0, new EntryProperties(4, 0, 19, 8), new Widget(BuiltInWidgets.kTextView));
        
        m_coast = createBooleanEntry("Coast Stuff", false, new EntryProperties(0, 2, 4, 4), new Widget(BuiltInWidgets.kToggleSwitch));

        //m_field = createSendableEntry("Field", m_field2d, new EntryProperties(null, null));
    }

    @Override
    public void periodic() {
        /* Neutral Modes */
        if(m_coast.getBoolean(false) != m_lastNeutralMode){
            m_neutralModeChanger.reset();
        }

        m_neutralModeChanger.runOnce(
            () -> {
                elevator.setNeutralMode(m_coast.getBoolean(false) ? NeutralModeValue.Coast : NeutralModeValue.Brake);
                pivot.setNeutralMode(m_coast.getBoolean(false) ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            }
        );

        m_field2d.setRobotPose(RobotContainer.getSwerve().getEstimatedPose());
        
        m_lastNeutralMode = m_coast.getBoolean(false);

        //m_matchTimeEntry.setDouble(DriverStation.getMatchTime());
    }

    public static SendableChooser<Command> getAutoChooser(){
        return m_autoChooser;
    }

    public void checkNulls(Swerve swerve, Elevator elevator, Pivot pivot){
        if(swerve == null){
            swerve = new Swerve();
            DriverStation.reportError("[Dashboard] Swerve not present, instantiating new", true);
        }
        if(elevator == null){
            swerve = new Swerve();
            DriverStation.reportError("[Dashboard] Elevator not present, instantiating new", true);
        }
        if(pivot == null){
            swerve = new Swerve();
            DriverStation.reportError("[Dashboard] Pivot not present, instantiating new", true);
        }
    }
}
