package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.device.motor.drive.DriveTrainConstants;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;

import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the
// information needed to recognize AprilTags on the backdrop during Autonomous.
public class BackdropParametersXML {
    public static final String TAG = BackdropParametersXML.class.getSimpleName();
    private static final String BACKDROP_FILE_NAME = "BackdropParameters.xml";

    private final Document document;
    private final XPath xpath;

    public BackdropParametersXML(String pXMLDir) {
        try {
            RobotLogCommon.c(TAG, "Parsing BackdropParameters.xml");

            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + BACKDROP_FILE_NAME));
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

    public BackdropParameters getBackdropParameters() throws XPathExpressionException {
        XPathExpression expr;

        // Point to the first node.
        expr = xpath.compile("//backdrop_parameters");
        Node backdrop_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (backdrop_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//backdrop_parameters' not found");

        // Point to <direction>
        Node direction_node = backdrop_parameters_node.getFirstChild();
        direction_node = XMLUtils.getNextElement(direction_node);
        if (direction_node == null || !direction_node.getNodeName().equals("direction") || direction_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'direction' not found or invalid");

        DriveTrainConstants.Direction direction =
                DriveTrainConstants.Direction.valueOf(direction_node.getTextContent().toUpperCase());

        // Strafe and distance adjustment percentages should be in the range
        // of > -100.0 and < +100.0.
        // <strafe_adjustment_percent>
        Node strafe_adjustment_node = direction_node.getNextSibling();
        strafe_adjustment_node = XMLUtils.getNextElement(strafe_adjustment_node);
        if (strafe_adjustment_node == null || !strafe_adjustment_node.getNodeName().equals("strafe_adjustment_percent") ||
                strafe_adjustment_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'strafe_adjustment_percent' not found");

        String strafeAdjustmentText = strafe_adjustment_node.getTextContent();
        double strafe_adjustment_percent;
        try {
            strafe_adjustment_percent = Double.parseDouble(strafeAdjustmentText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'strafe_adjustment_percent'");
        }

        if (strafe_adjustment_percent < -100.0 || strafe_adjustment_percent > 100.0) {
            throw new AutonomousRobotException(TAG, "Element 'strafe_adjustment_percent' must be > -100.0 and < +100.0");
        }

        strafe_adjustment_percent /= 100.0;

        // <distance_adjustment_percent>
        Node distance_adjustment_node = strafe_adjustment_node.getNextSibling();
        distance_adjustment_node = XMLUtils.getNextElement(distance_adjustment_node);
        if (distance_adjustment_node == null || !distance_adjustment_node.getNodeName().equals("distance_adjustment_percent") ||
                distance_adjustment_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'distance_adjustment_percent' not found");

        String distanceAdjustmentText = distance_adjustment_node.getTextContent();
        double distance_adjustment_percent;
        try {
            distance_adjustment_percent = Double.parseDouble(distanceAdjustmentText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'distance_adjustment_percent'");
        }

        if (distance_adjustment_percent < -100.0 || distance_adjustment_percent > 100.0) {
            throw new AutonomousRobotException(TAG, "Element 'distance_adjustment_percent' must be > -100.0 and < +100.0");
        }

        distance_adjustment_percent /= 100.0;

        // Parse <outside_strafe_adjustment>.
        Node outside_strafe_node = distance_adjustment_node.getNextSibling();
        outside_strafe_node = XMLUtils.getNextElement(outside_strafe_node);
        if (outside_strafe_node == null || !outside_strafe_node.getNodeName().equals("outside_strafe_adjustment") ||
                outside_strafe_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'outside_strafe_adjustment' not found");

        String outsideStrafeText = outside_strafe_node.getTextContent();
        double outside_strafe;
        try {
            outside_strafe = Double.parseDouble(outsideStrafeText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'outside_strafe_adjustment'");
        }

        if (outside_strafe < 0 || outside_strafe > 5.0) {
            throw new AutonomousRobotException(TAG, "Element 'outside_strafe_adjustment' out of range");
        }

        // Parse <yellow_pixel_adjustment>.
        Node yellow_pixel_node = outside_strafe_node.getNextSibling();
        yellow_pixel_node = XMLUtils.getNextElement(yellow_pixel_node);
        if (yellow_pixel_node == null || !yellow_pixel_node.getNodeName().equals("yellow_pixel_adjustment") ||
                yellow_pixel_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'yellow_pixel_adjustment' not found");

        String yellowPixelText = yellow_pixel_node.getTextContent();
        double yellowPixelAdjustment;
        try {
            yellowPixelAdjustment = Double.parseDouble(yellowPixelText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'yellow_pixel_adjustment'");
        }

        return new BackdropParameters(direction,
                strafe_adjustment_percent, distance_adjustment_percent,
                outside_strafe, yellowPixelAdjustment);
    }

}

