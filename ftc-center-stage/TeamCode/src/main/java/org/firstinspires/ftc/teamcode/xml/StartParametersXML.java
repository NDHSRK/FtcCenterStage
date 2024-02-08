package org.firstinspires.ftc.teamcode.xml;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.EnumMap;
import java.util.Objects;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class StartParametersXML {

    public static final String TAG = StartParametersXML.class.getSimpleName();
    private static final String FILE_NAME = "StartParameters.xml";
    private final String xmlDirectory;
    private final String xmlFilePath;
    private final Document document;

    private final Node delay_node; // save the DOM Node for update
    private int autoStartDelay;
    private final EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions
            = new EnumMap<>(RobotConstantsCenterStage.OpMode.class);

    // Save the DOM Nodes for update.
    private final EnumMap<RobotConstantsCenterStage.OpMode, Node> autoEndingPositionNodes
            = new EnumMap<>(RobotConstantsCenterStage.OpMode.class);

    // Optional
    private Node pathNode;
    private Node postSpikeDelayMsNode;
    private Node preBackstageDelayMsNode;

    // Common
    private final StartParameters startParameters;

    // IntelliJ only
    /*
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */
    // End IntelliJ only

    public StartParametersXML(String pXMLDirectory) throws ParserConfigurationException, SAXException, IOException {

        // IntelliJ only
        /*
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser.
        dbFactory.setIgnoringElementContentWhitespace(true);
         */
        // End IntelliJ only

        // Android only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android only

        xmlDirectory = pXMLDirectory;
        xmlFilePath = xmlDirectory + FILE_NAME;

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        document = dBuilder.parse(new File(xmlFilePath));
        Element startParametersRoot = document.getDocumentElement();

        Node config_node = startParametersRoot.getFirstChild();
        config_node = XMLUtils.getNextElement(config_node);
        if (config_node == null || !config_node.getNodeName().equals("robot_config") || config_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_config' not found");

        String robotConfigFilename = xmlDirectory + config_node.getTextContent();

        Node action_node = config_node.getNextSibling();
        action_node = XMLUtils.getNextElement(action_node);
        if (action_node == null || !action_node.getNodeName().equals("robot_action") || action_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_action' not found");

        String robotActionFilename = xmlDirectory + action_node.getTextContent();

        Node local_delay_node = action_node.getNextSibling();
        local_delay_node = XMLUtils.getNextElement(local_delay_node);
        if (local_delay_node == null || !local_delay_node.getNodeName().equals("auto_start_delay") || local_delay_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'auto_start_delay' not found");

        String delayString = local_delay_node.getTextContent();
        try {
            autoStartDelay = Integer.parseInt(delayString);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'auto_start_delay'");
        }

        delay_node = local_delay_node; // preserve "final"

        // Parse the <auto_ending_position> element.
        Node ending_position_node = local_delay_node.getNextSibling();
        ending_position_node = XMLUtils.getNextElement(ending_position_node);
        if (ending_position_node == null || !ending_position_node.getNodeName().equals("auto_ending_position"))
            throw new AutonomousRobotException(TAG, "Element 'auto_ending_position' not found");

        // Parse the four children of the <auto_ending_position> element,
        // one for each Autonomous competition OpMode.
        NodeList ending_position_elements = ending_position_node.getChildNodes();
        if (ending_position_elements == null)
            throw new AutonomousRobotException(TAG, "Missing 'auto_ending_position' elements");

        XMLUtils.processElements(ending_position_elements, (opmode_node) -> {
            RobotConstantsCenterStage.OpMode autoOpMode = RobotConstantsCenterStage.OpMode.valueOf(opmode_node.getNodeName());
            String endingPositionText = opmode_node.getTextContent().toUpperCase().trim();
            RobotConstantsCenterStage.AutoEndingPosition endingPosition = RobotConstantsCenterStage.AutoEndingPosition.valueOf(endingPositionText);

            if (autoEndingPositions.put(autoOpMode, endingPosition) != null)
                throw new AutonomousRobotException(TAG, "Duplicate Autonomous OpMode");

            // Save the DOM Node for update.
            autoEndingPositionNodes.put(autoOpMode, opmode_node);
        });

        // Parse the optional <optional_start_parameters>
        StartParameters.OptionalStartParameters optionalStartParameters = null;
        Node optional_node = ending_position_node.getNextSibling();
        if (optional_node != null) {
            optional_node = XMLUtils.getNextElement(optional_node);
            if (optional_node == null || !optional_node.getNodeName().equals("optional_start_parameters"))
                throw new AutonomousRobotException(TAG, "Element 'optional_start_parameters' not found");

            pathNode = optional_node.getFirstChild();
            pathNode = XMLUtils.getNextElement(pathNode);
            if (pathNode == null || !pathNode.getNodeName().equals("path") || pathNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'path' not found");

            String pathString = pathNode.getTextContent().toUpperCase().trim();
            StartParameters.OptionalStartParameters.Path path = StartParameters.OptionalStartParameters.Path.valueOf(pathString);

            // Parse two <mid_path_delay> elements. The first must contain a first child of
            // element <delay_point>post_spike</delay_point>; the second must contain a
            // first child of <<delay_point>pre_backstage</delay_point>.
            Node mid_path_delay_spike_node = pathNode.getNextSibling();
            mid_path_delay_spike_node = XMLUtils.getNextElement(mid_path_delay_spike_node);
            Pair<Node, Integer> midPathDelayPostSpike = parseMidPathDelay(mid_path_delay_spike_node, StartParameters.OptionalStartParameters.MidPathDelayPoint.POST_SPIKE);
            postSpikeDelayMsNode = midPathDelayPostSpike.first;

            Node mid_path_delay_backstage_node = mid_path_delay_spike_node.getNextSibling();
            mid_path_delay_backstage_node = XMLUtils.getNextElement(mid_path_delay_backstage_node);
            Pair<Node, Integer> midPathDelayPreBackstage = parseMidPathDelay(mid_path_delay_backstage_node, StartParameters.OptionalStartParameters.MidPathDelayPoint.PRE_BACKSTAGE);
            preBackstageDelayMsNode = midPathDelayPreBackstage.first;
            optionalStartParameters = new StartParameters.OptionalStartParameters(path, midPathDelayPostSpike.second, midPathDelayPreBackstage.second);
        }

        startParameters = new StartParameters(robotConfigFilename, robotActionFilename, autoStartDelay, autoEndingPositions, optionalStartParameters);
        RobotLog.ii(TAG, "In StartParametersXML; opened and parsed the XML file");
    }

    // Warning: this method returns the start parameters from the XML
    // file. Any changes made to the parameters by any "set" methods
    // below are not reflected in the startParameters variable. If you
    // do want to reflect the changes you will have to include:
    //  startParameters = new StartParameters(robotConfigFilename, robotActionFilename, autoStartDelay, autoEndingPositions);
    // after every change.
    public StartParameters getStartParameters() {
        return startParameters;
    }

    // Replaces the text value of the <auto_start_delay> element.
    public void setAutoStartDelay(int pAutoStartDelay) {
        RobotLog.ii(TAG, "Setting the Autonomous start delay in startParameters to " + pAutoStartDelay);
        autoStartDelay = pAutoStartDelay;
        delay_node.setTextContent(Integer.toString(pAutoStartDelay));
    }

    public void setAutoEndingPosition(RobotConstantsCenterStage.OpMode pAutoOpMode, String pEndingPositionText) {
        String endingPositionText = pEndingPositionText.toUpperCase().trim();
        RobotConstantsCenterStage.AutoEndingPosition endingPosition = RobotConstantsCenterStage.AutoEndingPosition.valueOf(endingPositionText);
        RobotLog.ii(TAG, "Setting ending position " + endingPositionText + " for Autonomous OpMode " + pAutoOpMode + " in in startParameters");

        autoEndingPositions.put(pAutoOpMode, endingPosition);
        Node endingPositionNode = autoEndingPositionNodes.get(pAutoOpMode);
        Objects.requireNonNull(endingPositionNode, TAG + " No ending position for OpMode " + pAutoOpMode).setTextContent(endingPositionText);
    }

    public void setOptionalPath(StartParameters.OptionalStartParameters.Path pPath) {
        pathNode.setTextContent(pPath.toString());
    }

    public void setOptionalDelayPoint(StartParameters.OptionalStartParameters.MidPathDelayPoint pDelayPoint,
                                    int pDelayMs) {
        switch (pDelayPoint) {
            case POST_SPIKE: {
                postSpikeDelayMsNode.setTextContent(Integer.toString(pDelayMs));
                break;
            }
            case PRE_BACKSTAGE: {
                preBackstageDelayMsNode.setTextContent(Integer.toString(pDelayMs));
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Invalid delay_point");
        }
    }

    public void writeStartParametersFile() {
        XMLUtils.writeXMLFile(document, xmlFilePath, xmlDirectory + RobotConstants.XSLT_FILE_NAME);
    }

    private Pair<Node, Integer> parseMidPathDelay(Node pMidPathDelayNode, StartParameters.OptionalStartParameters.MidPathDelayPoint pDelayPoint) {
        if (pMidPathDelayNode == null || !pMidPathDelayNode.getNodeName().equals("mid_path_delay"))
            throw new AutonomousRobotException(TAG, "Element 'mid_path_delay' not found");

        Node delay_point_node = pMidPathDelayNode.getFirstChild();
        delay_point_node = XMLUtils.getNextElement(delay_point_node);
        if (delay_point_node == null || !delay_point_node.getNodeName().equals("delay_point") || delay_point_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'delay_point' not found");

        String delayPointString = delay_point_node.getTextContent().toUpperCase().trim();
        StartParameters.OptionalStartParameters.MidPathDelayPoint delayPoint =
                StartParameters.OptionalStartParameters.MidPathDelayPoint.valueOf(delayPointString);

        if (delayPoint != pDelayPoint)
            throw new AutonomousRobotException(TAG, "Missing expected delay point " + pDelayPoint);

        Node delay_ms_node = delay_point_node.getNextSibling();
        delay_ms_node = XMLUtils.getNextElement(delay_ms_node);
        if (delay_ms_node == null || !delay_ms_node.getNodeName().equals("delay_ms") || delay_ms_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'delay_ms' not found");

        String delayMsString = delay_ms_node.getTextContent();
        int delayMs;
        try {
            delayMs = Integer.parseInt(delayMsString);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'delay_ms'");
        }

        return Pair.create(delay_ms_node, delayMs);
    }

}
