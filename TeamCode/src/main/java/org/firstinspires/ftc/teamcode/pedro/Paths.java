package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths{

    public final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startPos = poseFactory.of(85.082, 132.8817, 90);
    public final Pose shoot1 = poseFactory.of(84.8377, 107.6054, 270);
    public final Pose collectGardenPart1 = poseFactory.of(119.3584, 132.9845, 0);
    public final Pose collectGardenPart1Control1 = poseFactory.of(98.2245, 124.082, 0);
    public final Pose collectGardenPart2 = poseFactory.of(132.6304, 133.0665, 0);
    public final Pose shoot2 = poseFactory.of(84.6459, 107.4188, 270);
    public final Pose shoot2Control1 = poseFactory.of(97.8903, 125.0242, 0);
    public final Pose collectFlower = poseFactory.of(131.9119, 93.6183, 0);
    public final Pose shoot3 = poseFactory.of(84.9223, 107.5959, 270);
    public final Pose park = poseFactory.of(132.2288, 44.9007, 180);
    public final Pose parkControl1 = poseFactory.of(132.4007, 93.0769, 0);

    private Path shoot1() {
        return line(startPos, shoot1).linear(startPos, shoot1);
    }

    private Path collectGardenPart1() {
        return curve(shoot1, collectGardenPart1Control1, collectGardenPart1).linear(shoot1, collectGardenPart1);
    }

    private Path collectGardenPart2() {
        return line(collectGardenPart1, collectGardenPart2).linear(collectGardenPart1, collectGardenPart2);
    }

    private Path shoot2() {
        return curve(collectGardenPart2, shoot2Control1, shoot2).linear(collectGardenPart2, shoot2);
    }

    private Path collectFlower() {
        return line(shoot2, collectFlower).linear(shoot2, collectFlower);
    }

    private Path shoot3() {
        return line(collectFlower, shoot3).linear(collectFlower, shoot3);
    }

    private Path park() {
        return curve(shoot3, parkControl1, park).linear(shoot3, park);
    }

}