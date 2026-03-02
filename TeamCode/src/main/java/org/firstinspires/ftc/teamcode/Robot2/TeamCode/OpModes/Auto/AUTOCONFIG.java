package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot2.Utils.SelectableOpMode;

@Autonomous (name = "Auto Config", group = "Auto")
public class AUTOCONFIG extends SelectableOpMode {

    public AUTOCONFIG() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Blue", l -> {
                l.add("Close 12 bile", BlueClose12ball::new);
                l.add("Close 12 bile, gate dupa preload", BlueClose12BallOpenGateAfterPreload::new);
                l.add("Far 6 bile", BlueCloseNoGate::new);
                l.add("Far bile human", BlueFarHuman::new);
                l.add("Far bile human", BlueFarSpikeAndHuman::new);
            });
            s.folder("Red", a -> {
                a.add("Close 12 bile", RedClose12ball::new);
                a.add("Close 12 bile, gate dupa preload", RedClose12BallOpenGateAfterPreload::new);
                a.add("Far 6 bile", RedCloseNoGate::new);
                a.add("Far bile human", RedFarHuman::new);
                a.add("Far bile human", RedFarSpikeAndHuman::new);
            });

        });
    }
}
