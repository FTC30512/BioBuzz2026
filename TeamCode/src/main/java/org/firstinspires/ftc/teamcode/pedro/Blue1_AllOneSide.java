package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


@Autonomous
public class Blue1_AllOneSide extends OpMode {

    private Follower follower;
    private Paths paths;

    public Path shoot1() {
        return line(paths.startPos, paths.shoot1).linear(paths.startPos, paths.shoot1);
    }

    public Path collectGardenPart1() {
        return curve(paths.shoot1, paths.collectGardenPart1Control1, paths.collectGardenPart1).linear(paths.shoot1, paths.collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return line(paths.collectGardenPart1, paths.collectGardenPart2).linear(paths.collectGardenPart1, paths.collectGardenPart2);
    }

    public Path shoot2() {
        return curve(paths.collectGardenPart2, paths.shoot2Control1, paths.shoot2).linear(paths.collectGardenPart2, paths.shoot2);
    }

    public Path collectFlower() {
        return line(paths.shoot2, paths.collectFlower).linear(paths.shoot2, paths.collectFlower);
    }

    public Path shoot3() {
        return line(paths.collectFlower, paths.shoot3).linear(paths.collectFlower, paths.shoot3);
    }

    public Path park() {
        return curve(paths.shoot3, paths.parkControl1, paths.park).linear(paths.shoot3, paths.park);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2()),
                follow(follower, collectFlower()),
                follow(follower, shoot3()),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        paths = new Paths();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPos);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
