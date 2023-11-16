package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.opencv.core.Rect;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

import java.io.File;
import java.io.IOException;
import java.util.EnumMap;

// The purpose of this class is to collect information from the
// FIND_TEAM_PROP element of each of the Autonomous OpModes in
// RobotConfig.xml associated with a starting position in the
// competition, e.g. BLUE_A2, RED_F4. The collected information
// can be used to crop webcam images and show spike mark windows
// can be shown in the Driver Station camera stream. Before a
// match the driver can then adjust the camera alignment.
public class SpikeWindowMappingXML {

    public static final String TAG = SpikeWindowMappingXML.class.getSimpleName();
    private static final String FILE_NAME = "RobotAction.xml";

    private final Document document;
    private final XPath xpath;

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    public SpikeWindowMappingXML(String pWorkingDirectory) throws ParserConfigurationException, SAXException, IOException {

    /*
    // IntelliJ only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser (DTD or schema),
        // which the IntelliJ parser is.
        dbFactory.setIgnoringElementContentWhitespace(true);
    // End IntelliJ only
    */

        // Android or IntelliJ
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        //## ONLY works with a validating parser (DTD or schema),
        // which the Android Studio parser is not.
        // dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android or IntelliJ

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        document = dBuilder.parse(new File(pWorkingDirectory + FILE_NAME));

        XPathFactory xpathFactory = XPathFactory.newInstance();
        xpath = xpathFactory.newXPath();
    }

    // Collect spike window data (resolution, ROI, spike windows, etc.)
    // for one Autonomous OpMode from RobotAction.xml. May return null
    // if the OpMode does not contain a <FIND_TEAM_PROP> element.
    public SpikeWindowData collectSpikeWindowData(RobotConstantsCenterStage.OpMode pOpMode) throws XPathExpressionException {
        RobotLogCommon.c(TAG, "Collecting Team Prop data for Autonomous OpMode " + pOpMode);
        return getSpikeWindowData(pOpMode);
    }

    // Collect spike window data (resolution, ROI, spike windows, etc.)
    // for all Autonomous competition OpModes from RobotAction.xml.
    public EnumMap<RobotConstantsCenterStage.OpMode, SpikeWindowData> collectSpikeWindowData() throws XPathExpressionException {
        EnumMap<RobotConstantsCenterStage.OpMode, SpikeWindowData> spikeWindowData =
                new EnumMap<>(RobotConstantsCenterStage.OpMode.class);

        // Get all OpModes but only process those with an OpModeType
        // of COMPETITION or AUTO_TEST.
        SpikeWindowMappingXML.SpikeWindowData spikeDataOneOpMode;
        RobotConstantsCenterStage.OpMode[] allOpModes =
                RobotConstantsCenterStage.OpMode.values();
        for (RobotConstantsCenterStage.OpMode oneOpMode : allOpModes) {
            if (oneOpMode.getOpModeType() == RobotConstantsCenterStage.OpMode.OpModeType.COMPETITION ||
                    oneOpMode.getOpModeType() == RobotConstantsCenterStage.OpMode.OpModeType.AUTO_TEST) {
                RobotLogCommon.c(TAG, "Collecting Team Prop data for Autonomous OpMode " + oneOpMode);
                spikeDataOneOpMode = getSpikeWindowData(oneOpMode);
                if (spikeDataOneOpMode != null)
                    spikeWindowData.put(oneOpMode, getSpikeWindowData(oneOpMode));
            }
        }

        return spikeWindowData;
    }

    // Find the requested opMode in the RobotAction.xml file.
    // Package and return all data associated with the
    // FIND_TEAM_PROP element under the OpMode.
    // Example:
    /*
        <actions>
            ...

            <FIND_TEAM_PROP>
                <image_parameters>
                    <image_source>front_webcam</image_source>
                    <resolution>
                        <width>640</width>
                        <height>480</height>
                    </resolution>
                    <image_roi>
                        <x>62</x>
                        <y>150</y>
                        <width>492</width>
                        <height>225</height>
                    </image_roi>
                </image_parameters>
                <team_prop_recognition>
                    <recognition_path>color_channel_circles</recognition_path>
                    <!-- These values are relative to the image ROI -->
                    <!-- Note: the left_window may enclose the LEFT_SPIKE
                         or the CENTER_SPIKE, depending on the robot's
                         starting position. For RED_F2 the left_window
                         encloses the LEFT_SPIKE. -->
                    <left_window>
                        <x>0</x>
                        <y>0</y>
                        <width>160</width>
                        <height>225</height>
                        <prop_location>LEFT_SPIKE</prop_location>
                    </left_window>
                    <right_window>
                        <!-- x starts at left_window.x + left_window.width -->
                        <width>330</width>
                        <!-- y is the same as that of the left_window -->
                        <!-- height is the same as that of the left_window -->
                        <prop_location>CENTER_SPIKE</prop_location>
                    </right_window>
                    <team_prop_npos>
                        <prop_location>RIGHT_SPIKE</prop_location>
                    </team_prop_npos>
                </team_prop_recognition>
            </FIND_TEAM_PROP>
           ...

         </actions>
     */
    private SpikeWindowData getSpikeWindowData(RobotConstantsCenterStage.OpMode pOpMode) throws XPathExpressionException {
        EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> spikeWindows =
                new EnumMap<>(RobotConstantsCenterStage.SpikeLocationWindow.class);

        // Use XPath to locate the desired OpMode and its child element FIND_TEAM_PROP.
        String findTeamPropPath = "/RobotAction/OpMode[@id=" + "'" + pOpMode + "']" + "/actions/FIND_TEAM_PROP";
        Node find_team_propNode = (Node) xpath.evaluate(findTeamPropPath, document, XPathConstants.NODE);
        if (find_team_propNode == null) {
            // RobotLogCommon.d(TAG, "No path to " + pOpMode + "/FIND_TEAM_PROP");
            return null;
        }

        RobotLogCommon.c(TAG, "Extracting data from RobotAction.xml for " + findTeamPropPath);

        // The next element in the XML is required: <image_parameters>
        Node image_node = find_team_propNode.getFirstChild();
        image_node = XMLUtils.getNextElement(image_node);
        if ((image_node == null) || !image_node.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'image_parameters' not found");

        VisionParameters.ImageParameters imageParameters = ImageXML.parseImageParameters(image_node);

        // Parse the children of the <team_prop_recognition> element.
        Node recognition_node = image_node.getNextSibling();
        recognition_node = XMLUtils.getNextElement(recognition_node);
        if ((recognition_node == null) || !recognition_node.getNodeName().equals("team_prop_recognition"))
            throw new AutonomousRobotException(TAG, "Element 'team_prop_recognition' not found");

        // Drop down and skip the <recognition_path> element.
        Node path_node = recognition_node.getFirstChild();
        path_node = XMLUtils.getNextElement(path_node);
        if ((path_node == null) || !path_node.getNodeName().equals("recognition_path"))
            throw new AutonomousRobotException(TAG, "Element 'recognition_path' not found");

        // Parse the <left_window> element.
        Node left_node = path_node.getNextSibling();
        left_node = XMLUtils.getNextElement(left_node);
        if ((left_node == null) || !left_node.getNodeName().equals("left_window"))
            throw new AutonomousRobotException(TAG, "Element 'left_window' not found");

        // Drop down and parse the children of the <left_window>
        Node left_x_node = left_node.getFirstChild();
        left_x_node = XMLUtils.getNextElement(left_x_node);
        if ((left_x_node == null) || !left_x_node.getNodeName().equals("x") || left_x_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'left_window/x' not found");

        String leftXText = left_x_node.getTextContent();
        int leftX;
        try {
            leftX = Integer.parseInt(leftXText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'left_window/x'");
        }

        Node left_y_node = left_x_node.getNextSibling();
        left_y_node = XMLUtils.getNextElement(left_y_node);
        if ((left_y_node == null) || !left_y_node.getNodeName().equals("y") || left_y_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'left_window/y' not found");

        String leftYText = left_y_node.getTextContent();
        int leftY;
        try {
            leftY = Integer.parseInt(leftYText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'left_window/y'");
        }

        Node left_width_node = left_y_node.getNextSibling();
        left_width_node = XMLUtils.getNextElement(left_width_node);
        if ((left_width_node == null) || !left_width_node.getNodeName().equals("width") || left_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'left_window/width' not found");

        String leftWidthText = left_width_node.getTextContent();
        int leftWidth;
        try {
            leftWidth = Integer.parseInt(leftWidthText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'left_window/width'");
        }

        Node left_height_node = left_width_node.getNextSibling();
        left_height_node = XMLUtils.getNextElement(left_height_node);
        if ((left_height_node == null) || !left_height_node.getNodeName().equals("height") || left_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'left_window/height' not found");

        String leftHeightText = left_height_node.getTextContent();
        int leftHeight;
        try {
            leftHeight = Integer.parseInt(leftHeightText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'left_window/height'");
        }

        // Parse the <prop_location> element.
        Node left_prop_node = left_height_node.getNextSibling();
        left_prop_node = XMLUtils.getNextElement(left_prop_node);
        if ((left_prop_node == null) || !left_prop_node.getNodeName().equals("prop_location") || left_prop_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'left_window/prop_location' not found");

        String leftPropLocationText = left_prop_node.getTextContent().toUpperCase();
        RobotConstantsCenterStage.TeamPropLocation leftPropLocation =
                RobotConstantsCenterStage.TeamPropLocation.valueOf(leftPropLocationText);

        spikeWindows.put(RobotConstantsCenterStage.SpikeLocationWindow.LEFT, Pair.create(new Rect(leftX, leftY, leftWidth, leftHeight), leftPropLocation));

        // Parse the <right_window> element.
        Node right_node = left_node.getNextSibling();
        right_node = XMLUtils.getNextElement(right_node);
        if ((right_node == null) || !right_node.getNodeName().equals("right_window"))
            throw new AutonomousRobotException(TAG, "Element 'right_window' not found");

        Node right_width_node = right_node.getFirstChild();
        right_width_node = XMLUtils.getNextElement(right_width_node);
        if ((right_width_node == null) || !right_width_node.getNodeName().equals("width") || right_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'right_window/width' not found");

        String rightWidthText = right_width_node.getTextContent();
        int rightWidth;
        try {
            rightWidth = Integer.parseInt(rightWidthText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'right_window/width'");
        }

        // Parse the <prop_location> element.
        Node right_prop_node = right_width_node.getNextSibling();
        right_prop_node = XMLUtils.getNextElement(right_prop_node);
        if ((right_prop_node == null) || !right_prop_node.getNodeName().equals("prop_location") || right_prop_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'right_window/prop_location' not found");

        String rightPropLocationText = right_prop_node.getTextContent().toUpperCase();
        RobotConstantsCenterStage.TeamPropLocation rightPropLocation =
                RobotConstantsCenterStage.TeamPropLocation.valueOf(rightPropLocationText);

        // Note: the right window starts 1 pixel past the left element. The height of the right
        // window is the same as that of the left window.
        spikeWindows.put(RobotConstantsCenterStage.SpikeLocationWindow.RIGHT, Pair.create(new Rect(leftX + leftWidth, leftY, rightWidth, leftHeight), rightPropLocation));

        // Parse the <team_prop_npos> element.
        Node npos_node = right_node.getNextSibling();
        npos_node = XMLUtils.getNextElement(npos_node);
        if ((npos_node == null) || !npos_node.getNodeName().equals("team_prop_npos"))
            throw new AutonomousRobotException(TAG, "Element 'team_prop_npos' not found");

        // Drop down and parse the <prop_location> element.
        Node npos_prop_node = npos_node.getFirstChild();
        npos_prop_node = XMLUtils.getNextElement(npos_prop_node);
        if ((npos_prop_node == null) || !npos_prop_node.getNodeName().equals("prop_location") || npos_prop_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'team_prop_npos/prop_location' not found");

        String nposLocationText = npos_prop_node.getTextContent().toUpperCase();
        RobotConstantsCenterStage.TeamPropLocation nposLocation =
                RobotConstantsCenterStage.TeamPropLocation.valueOf(nposLocationText);

        spikeWindows.put(RobotConstantsCenterStage.SpikeLocationWindow.WINDOW_NPOS, Pair.create(new Rect(0, 0, 0, 0), nposLocation));

        return new SpikeWindowData(imageParameters, spikeWindows);
    }

    public static class SpikeWindowData {
        public final VisionParameters.ImageParameters imageParameters;
        public final EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> spikeWindows;

        public SpikeWindowData(VisionParameters.ImageParameters pImageParameters,
                               EnumMap<RobotConstantsCenterStage.SpikeLocationWindow, Pair<Rect, RobotConstantsCenterStage.TeamPropLocation>> pSpikeWindows) {
            imageParameters = pImageParameters;
            spikeWindows = pSpikeWindows;
        }
    }

}