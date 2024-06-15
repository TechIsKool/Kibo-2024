package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class Coordinates extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // The mission starts.
        api.startMission();

        // Area 1 coords
        Point area1 = new Point(10.9d, -9.92284d, 5.195d);
        Quaternion quaternion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(area1, quaternion1, false);

        // Area 2p1 coords
        Point area2p1 = new Point(11.235d, -9.45d, 5.295d);
        Quaternion quaternion2p1 = new Quaternion(1, 0, 0, 0);
        api.moveTo(area2p1, quaternion2p1, true);

        // Area 2p2 coords
        Point area2p2 = new Point(10.925d, -8.875d, 4.26203d);
        Quaternion quaternion2p2 = new Quaternion(1, 0, 0, 0);
        api.moveTo(area2p2, quaternion2p2, true);

        // KOZ 2
        //Point area2p2 = new Point(10.475d, -8.45d, 5.295d);
        //Quaternion quaternion2p2 = new Quaternion(1, 0, 0, 0);
        // api.moveTo(area2p2, quaternion2p2, true);

        // Get a camera image.
        Mat image = api.getMatNavCam();

        /* *********************************************************************** */
        /* Write your code to recognize type and number of items in the each area! */
        /* *********************************************************************** */

        // When you recognize items, let’s set the type and number.
        api.setAreaInfo(1, "item_name", 1);

        /* **************************************************** */
        /* Let's move to the each area and recognize the items. */
        /* **************************************************** */

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here.
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
    }

    // You can add your method.
    private String yourMethod(){
        return "your method";
    }
}