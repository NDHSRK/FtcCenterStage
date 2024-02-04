package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.logging.Level;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

public class RobotActionXMLCenterStage {

    public static final String TAG = RobotActionXMLCenterStage.class.getSimpleName();

    private final Document document;
    private final XPath xpath;

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    public RobotActionXMLCenterStage(String pRobotActionFilename) throws ParserConfigurationException, SAXException, IOException {

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
        document = dBuilder.parse(new File(pRobotActionFilename));

        XPathFactory xpathFactory = XPathFactory.newInstance();
        xpath = xpathFactory.newXPath();
    }

    // Find the requested opMode in the RobotAction.xml file.
    // Package and return all data associated with the OpMode.
    public RobotActionDataCenterStage getOpModeData(String pOpMode) throws XPathExpressionException {

        Level logLevel = null; // null means use the default lowest logging level
        StartingPositionData startingPositionData = null;
        List<RobotXMLElement> actions = new ArrayList<>();

        // Use XPath to locate the desired OpMode.
        String opModePath = "/RobotAction/OpMode[@id=" + "'" + pOpMode + "']";
        Node opModeNode = (Node) xpath.evaluate(opModePath, document, XPathConstants.NODE);
        if (opModeNode == null)
            throw new AutonomousRobotException(TAG, "Missing OpMode " + pOpMode);

        RobotLogCommon.c(TAG, "Extracting data from RobotAction.xml for OpMode " + pOpMode);

        // The next element in the XML is required: <parameters>
        Node parametersNode = XMLUtils.getNextElement(opModeNode.getFirstChild());
        if ((parametersNode == null) || !parametersNode.getNodeName().equals("parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <parameters> element");

        // The possible elements under <parameters> are:
        //   <log_level>
        //   <starting_position>
        // All are optional.

        // A missing or empty optional logging_level will eventually return null, which
        // means to use the logger's default.
        Node nextParameterNode = XMLUtils.getNextElement(parametersNode.getFirstChild());
        if ((nextParameterNode != null) && (nextParameterNode.getNodeName().equals("log_level"))) {
            String logLevelString = nextParameterNode.getTextContent().trim();
            if (!logLevelString.isEmpty()) {
                switch (logLevelString) {
                    case "off": {
                        logLevel = Level.OFF;
                        break;
                    }
                    case "i": { // highest
                        logLevel = Level.INFO;
                        break;
                    }
                    case "c": {
                        logLevel = Level.CONFIG;
                        break;
                    }
                    case "d": {
                        logLevel = Level.FINE;
                        break;
                    }
                    case "v": {
                        logLevel = Level.FINER;
                        break;
                    }
                    case "vv": {
                        logLevel = Level.FINEST;
                        break;
                    }
                    default: {
                        throw new AutonomousRobotException(TAG, "Invalid logging level");
                    }
                }
            }
            nextParameterNode = XMLUtils.getNextElement(nextParameterNode.getNextSibling());
        }

        // The next optional element in the XML is <starting_position>.
        if ((nextParameterNode != null) && nextParameterNode.getNodeName().equals("starting_position")) {
            // Get the value from each child of the starting_position element:
            // <x>79.0</x>
            // <y>188.0</y>
            // <angle>0.0</angle>
            double x;
            double y;
            double angle;
            Node xNode = XMLUtils.getNextElement(nextParameterNode.getFirstChild());
            if ((xNode == null) || !xNode.getNodeName().equals("x") || xNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'x' missing or empty");

            try {
                x = Double.parseDouble(xNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'x'");
            }

            Node yNode = XMLUtils.getNextElement(xNode.getNextSibling());
            if ((yNode == null) || !yNode.getNodeName().equals("y") || yNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'y' missing or empty");

            try {
                y = Double.parseDouble(yNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'y'");
            }

            Node angleNode = XMLUtils.getNextElement(yNode.getNextSibling());
            if ((angleNode == null) || !angleNode.getNodeName().equals("angle") || angleNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'angle' missing or empty");

            try {
                angle = Double.parseDouble(angleNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'angle'");
            }

            startingPositionData = new StartingPositionData(x, y, angle);
            nextParameterNode = XMLUtils.getNextElement(nextParameterNode.getNextSibling());
        }

        // Make sure there are no extraneous elements.
        if (nextParameterNode != null)
            throw new AutonomousRobotException(TAG, "Unrecognized element under <parameters>");

        // Now proceed to the <actions> element of the selected OpMode.
        String actionsPath = opModePath + "/actions";
        Node actionsNode = (Node) xpath.evaluate(actionsPath, document, XPathConstants.NODE);
        if (actionsNode == null)
            throw new AutonomousRobotException(TAG, "Missing <actions> element");

        // Now iterate through the children of the <actions> element of the selected OpMode.
        NodeList actionChildren = actionsNode.getChildNodes();
        Node actionNode;

        RobotXMLElement actionXMLElement;
        boolean teamPropLocationChoice = false;
        EnumMap<RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> teamPropLocationActions = null;
        for (int i = 0; i < actionChildren.getLength(); i++) {
            actionNode = actionChildren.item(i);

            if (actionNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            actionXMLElement = new RobotXMLElement((Element) actionNode);
            actions.add(actionXMLElement);

            if (actionNode.getNodeName().equals("TEAM_PROP_LOCATION_CHOICE")) {
                if (teamPropLocationChoice)
                    throw new AutonomousRobotException(TAG, "Only one TEAM_PROP_LOCATION_CHOICE element is allowed");
                teamPropLocationChoice = true;

                // Collect all the RobotXMLElement(s) for each team prop location.
                // The elements will be fed into the stream of actions at run-time
                // depending on the outcome of the team prop recognition
                teamPropLocationActions = TeamPropXMLUtils.getTeamPropLocationActions(actionNode);
            }
        }

        return new RobotActionDataCenterStage(logLevel, startingPositionData,
                actions, teamPropLocationActions);
    }

    // Helper method to convert a nested <image_parameters> element into a class.
    public VisionParameters.ImageParameters
    getImageParametersFromXPath(RobotXMLElement pElement, String pPath) throws XPathExpressionException {
        Node ipNode = (Node) xpath.evaluate(pPath, pElement.getRobotXMLElement(), XPathConstants.NODE);
        if (ipNode == null)
            throw new AutonomousRobotException(TAG, "Missing " + pPath + " element");

        if (!ipNode.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Expected <image_parameters> element");

        return ImageXML.parseImageParameters(ipNode);
    }

    public static class RobotActionDataCenterStage {
        public final Level logLevel;
        public final StartingPositionData startingPositionData;
        public final List<RobotXMLElement> actions;
        public final EnumMap<RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> teamPropLocationActions;

        public RobotActionDataCenterStage(Level pLogLevel,
                                          StartingPositionData pStartingPositionData,
                                          List<RobotXMLElement> pActions,
                                          EnumMap<RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> pTeamPropLocationActions) {
            logLevel = pLogLevel;
            startingPositionData = pStartingPositionData;
            actions = pActions;
            teamPropLocationActions = pTeamPropLocationActions;
        }
    }

    public static ArrayList<Pair<RobotConstantsCenterStage.ProcessorIdentifier, Boolean>> getStartWebcamProcessors(RobotXMLElement pStartWebcamElement) {

        // Get the <processor_set> child of the <START_WEBCAM>.
        NodeList processor_set_node = pStartWebcamElement.getRobotXMLElement().getElementsByTagName("processor_set");
        if (processor_set_node == null || processor_set_node.getLength() != 1)
            throw new AutonomousRobotException(TAG, "START_WEBCAM does not have exactly one processor_set child");

        // Process all of the <processor> children of the <processor_set> element.
        NodeList processor_nodes = processor_set_node.item(0).getChildNodes();
        if (processor_nodes == null)
            throw new AutonomousRobotException(TAG, "Element <processor_set> has no child elements");

        ArrayList<Pair<RobotConstantsCenterStage.ProcessorIdentifier, Boolean>> webcamProcessors = new ArrayList<>();

        XMLUtils.processElements(processor_nodes, (processor_node) -> {
            if (processor_node != null && processor_node.getNodeName().equals("processor")) {
                if (processor_node.getTextContent().isEmpty())
                    throw new AutonomousRobotException(TAG, "Element 'processor' is empty");

                // Look for the "disable_on_start" attribute; if it is present and
                // its value is "yes" then set the processor to be disabled when the
                // webcam starts.
                boolean enableOnStart = true; // default
                NamedNodeMap processor_attributes = processor_node.getAttributes();
                Node disable_node = processor_attributes.getNamedItem("disable_on_start");
                if (disable_node != null && disable_node.getTextContent().equals("yes"))
                    enableOnStart = false;

                RobotConstantsCenterStage.ProcessorIdentifier processorId =
                        RobotConstantsCenterStage.ProcessorIdentifier.valueOf(processor_node.getTextContent().toUpperCase());
                webcamProcessors.add(Pair.create(processorId, enableOnStart));
            }
        });

        // The collection of processors must not be empty.
        if (webcamProcessors.isEmpty())
            throw new AutonomousRobotException(TAG, "Element <processor_set> has no child <processor> elements");

        return webcamProcessors;
    }

    // Support the <AUTO_ENDING_POSITION> element in RobotAction.xml.
    // The first child of <AUTO_ENDING_POSITION> must be either
    // <left> or <right> and the grandchildren may be either
    // <STRAFE_LEFT> or <STRAFE_RIGHT>. Return the grandchild element
    // and the angle of the strafe direction: 90 degrees or -90 degrees
    // respectively.
    public static Pair<RobotXMLElement, Double> getFinalPositionElement(RobotXMLElement pElement, String pPosition) {
        Node left_position_node = pElement.getRobotXMLElement().getFirstChild();
        left_position_node = XMLUtils.getNextElement(left_position_node);
        if (left_position_node == null || !left_position_node.getNodeName().equals("LEFT"))
            throw new AutonomousRobotException(TAG, "The first child of <AUTO_ENDING_POSITION> must be <LEFT>");

        if (pPosition.equals("LEFT")) {
            // Step down to the <STRAFE_LEFT> or <STRAFE_RIGHT> element
            Node left_position_strafe_node = left_position_node.getFirstChild();
            left_position_strafe_node = XMLUtils.getNextElement(left_position_strafe_node);
            if (left_position_strafe_node == null || !(left_position_strafe_node.getNodeName().equals("STRAFE_LEFT") ||
                    left_position_strafe_node.getNodeName().equals("STRAFE_RIGHT")))
                throw new AutonomousRobotException(TAG, "The first child of AUTO_ENDING_POSITION/LEFT must be <STRAFE_LEFT> or <STRAFE_RIGHT>");

            double strafe_angle = left_position_strafe_node.getNodeName().equals("STRAFE_LEFT") ? 90.0 : -90.0;

            return Pair.create(new RobotXMLElement((Element) left_position_strafe_node), strafe_angle);
        }

        // Must be the right position.
        Node right_position_node = left_position_node.getNextSibling();
        right_position_node = XMLUtils.getNextElement(right_position_node);
        if (right_position_node == null || !right_position_node.getNodeName().equals("RIGHT"))
            throw new AutonomousRobotException(TAG, "The second child of <AUTO_ENDING_POSITION> must be <RIGHT>");

        // Step down to the <STRAFE_LEFT> or <STRAFE_RIGHT> element
        Node right_position_strafe_node = right_position_node.getFirstChild();
        right_position_strafe_node = XMLUtils.getNextElement(right_position_strafe_node);
        if (right_position_strafe_node == null || !(right_position_strafe_node.getNodeName().equals("STRAFE_LEFT") ||
                right_position_strafe_node.getNodeName().equals("STRAFE_RIGHT")))
            throw new AutonomousRobotException(TAG, "The first child of AUTO_ENDING_POSITION/LEFT must be <STRAFE_LEFT> or <STRAFE_RIGHT>");

        double strafe_angle = right_position_strafe_node.getNodeName().equals("STRAFE_LEFT") ? 90.0 : -90.0;

        return Pair.create(new RobotXMLElement((Element) right_position_strafe_node), strafe_angle);
    }

    public static class StartingPositionData {

        public final double startingX; // FTC field coordinates
        public final double startingY; // FTC field coordinates
        public final double startingAngle; // with respect to the wall

        public StartingPositionData(double pStartingX, double pStartingY, double pStartingAngle) {
            startingX = pStartingX;
            startingY = pStartingY;
            startingAngle = pStartingAngle;
        }
    }

}