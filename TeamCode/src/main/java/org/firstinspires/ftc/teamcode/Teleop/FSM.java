package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled
@TeleOp
public class FSM extends OpMode {
    enum state{
        WAIT_FOR_A,
        WAIT_FOR_B,
        WAIT_FOR_X,
        FINISHED
    }

    state State = state.WAIT_FOR_A;

    @Override
    public void init() {
        State = state.WAIT_FOR_A;
    }

    @Override
    public void loop() {
        telemetry.addData("Current State", State);
        switch (State){
            case WAIT_FOR_A:
                telemetry.addLine("To exit state. Press A");
                if(gamepad1.a){
                    State = state.WAIT_FOR_B;
                }
                break;
            case WAIT_FOR_B:
                telemetry.addLine("To exit state. Press B");
                if(gamepad1.b){
                    State = state.WAIT_FOR_X;
                }
                break;
            case WAIT_FOR_X:
                telemetry.addLine("To exit state. Press X");
                if(gamepad1.x){
                    State = state.FINISHED;
                }
                break;
            default:
                telemetry.addLine("FSM finished");
        }

    }
}
