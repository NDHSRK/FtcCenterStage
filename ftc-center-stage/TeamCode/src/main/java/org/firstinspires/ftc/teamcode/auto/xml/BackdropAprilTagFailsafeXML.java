package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

// For each of the 6 AprilTags located on the two backdrops provide
// a set of movements that the robot will make in the event of an
// AprilTag recogntion failure. The movements should attempt to take
// to a position opposite the selected AprilTag. 
public class BackdropAprilTagFailsafeXML {

    public static final String TAG = BackdropAprilTagFailsafeXML.class.getSimpleName();
    private static final String FILE_NAME = "BackdropAprilTagFailsafe.xml";

    private final Document document;
    private final XPath xpath;

    public BackdropAprilTagFailsafeXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + FILE_NAME));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

        } catch (ParserConfigurationException pex) {
            String eMessage = pex.getMessage() == null ? "**no error message**" : pex.getMessage();
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + eMessage);
        } catch (SAXException sx) {
            String eMessage = sx.getMessage() == null ? "**no error message**" : sx.getMessage();
            throw new AutonomousRobotException(TAG, "SAX Exception " + eMessage);
        } catch (IOException iex) {
            String eMessage = iex.getMessage() == null ? "**no error message**" : iex.getMessage();
            throw new AutonomousRobotException(TAG, "IOException " + eMessage);
        }
    }

    // This method is unlike the similar one in RobotActionXMLCenterSTage (getOpModeData)
    // in that here we parse all elements under the root <backdrop_apriltag_failsafe>
    // because we don't know in advance which will be needed as the robot moves through
    // its Autonomous choreography and encounters an AprilTag recognition problem.
    public EnumMap<RobotConstantsCenterStage.AprilTagId, List<RobotXMLElement>> getBackdropAprilTagFailsafeData() throws XPathExpressionException {

        RobotLogCommon.d(TAG, "Parsing XML BackdropAprilTagFailsafe");

        // Point to the first node.
        XPathExpression expr = xpath.compile("//backdrop_apriltag_failsafe");
        Node failsafe_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (failsafe_node == null)
            throw new AutonomousRobotException(TAG, "Element '//backdrop_apriltag_failsafe' not found");

        // Process all <TAG_ID_n> elements where "n" is 1 through 6.
        EnumMap<RobotConstantsCenterStage.AprilTagId, List<RobotXMLElement>> failsafeMap =
                new EnumMap<>(RobotConstantsCenterStage.AprilTagId.class);

        // The nodes must be named "TAG_ID_1" through "TAG_ID_6" in order.
        int tagIdNumber = 1;
        NodeList failsafeChildren = failsafe_node.getChildNodes();
        Node tag_id_node;
        for (int i = 0; i < failsafeChildren.getLength(); i++) {
            tag_id_node = failsafeChildren.item(i);

            if (tag_id_node.getNodeType() != Node.ELEMENT_NODE)
                continue;

            String tagIdString = "TAG_ID_" + new DecimalFormat("0").format(tagIdNumber++);

            if (!tag_id_node.getNodeName().equals("tagIdString"))
                throw new AutonomousRobotException(TAG, "Child element of 'backdrop_apriltag_failsafe' is not the expected " + tagIdString);

            RobotConstantsCenterStage.AprilTagId tagId = RobotConstantsCenterStage.AprilTagId.valueOf(tagIdString);

            // Get the children of the <TAG_ID_n> element.
            NodeList actions_node_list = tag_id_node.getChildNodes();
            List<RobotXMLElement> backdropFailsafeActions = collectBackdropFailsafeActions(actions_node_list);
            failsafeMap.put(tagId, backdropFailsafeActions);
        }

        return failsafeMap;
    }

    // Iterate through the children of a TAG_ID_n node and collect
    // the elements. During testing there may be no child elements.
    private static List<RobotXMLElement> collectBackdropFailsafeActions(NodeList pActionsNodeList) {

        List<RobotXMLElement> actions = new ArrayList<>();
        if (pActionsNodeList == null || pActionsNodeList.getLength() == 0)
            return actions; // nobody home

        Node oneActionNode;
        RobotXMLElement actionXMLElement;
        int nodeCount = pActionsNodeList.getLength();
        for (int i = 0; i < nodeCount; i++) {
            oneActionNode = pActionsNodeList.item(i);

            if (oneActionNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            actionXMLElement = new RobotXMLElement((Element) oneActionNode);
            actions.add(actionXMLElement);
        }

        return actions;
    }

}