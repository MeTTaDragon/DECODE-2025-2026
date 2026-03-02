package org.firstinspires.ftc.teamcode.Robot2.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class SelectableOpMode extends OpMode {
    private final Selector<Supplier<CommandOpMode>> selector;
    private CommandOpMode selectedOpMode;
    private final static String[] MESSAGE = {
            "Use the d-pad to move the cursor.",
            "Press right bumper or d-pad right to select.",
            "Press left bumper or d-pad left to go back."
    };

    public SelectableOpMode(String name, Consumer<SelectScope<Supplier<CommandOpMode>>> opModes) {
        selector = Selector.create(name, opModes, MESSAGE);
        selector.onSelect(opModeSupplier -> {
            onSelect();
            selectedOpMode = opModeSupplier.get();
            selectedOpMode.gamepad1 = gamepad1;
            selectedOpMode.gamepad2 = gamepad2;
            selectedOpMode.telemetry = telemetry;
            selectedOpMode.hardwareMap = hardwareMap;

            // why does the sdk have to suck so much
            final Field internalOpModeServices;
            try {
                internalOpModeServices = Objects.requireNonNull(OpMode.class.getSuperclass()).getDeclaredField("internalOpModeServices");
            } catch (NoSuchFieldException e) {
                throw new RuntimeException(e);
            }
            internalOpModeServices.setAccessible(true);
            try {
                internalOpModeServices.set(selectedOpMode, internalOpModeServices.get(this));
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
            selectedOpMode.initialize();
        });
    }

    protected void onSelect() {
    }

    protected void onLog(List<String> line) {
    }

    @Override
    public final void init() {
    }

    @Override
    public final void init_loop() {
        if (selectedOpMode == null) {
            if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
                selector.decrementSelected();
            else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
                selector.incrementSelected();
            else if (gamepad1.rightBumperWasPressed() ||
                    gamepad2.rightBumperWasPressed() ||
                    gamepad1.dpadRightWasPressed() ||
                    gamepad2.dpadRightWasPressed())
                selector.select();
            else if (gamepad1.leftBumperWasPressed() ||
                    gamepad2.leftBumperWasPressed() ||
                    gamepad1.dpadLeftWasPressed() ||
                    gamepad2.dpadLeftWasPressed())
                selector.goBack();

            List<String> lines = selector.getLines();
            for (String line : lines) {
                telemetry.addLine(line);
            }
            onLog(lines);
        } else selectedOpMode.initialize_loop();
    }

    @Override
    public final void start() {
        if (selectedOpMode == null) throw new RuntimeException("No OpMode selected!");
        selectedOpMode.run();
    }

    @Override
    public final void loop() {
    }

    @Override
    public final void stop() {
        if (selectedOpMode != null) selectedOpMode.end();
    }
}
