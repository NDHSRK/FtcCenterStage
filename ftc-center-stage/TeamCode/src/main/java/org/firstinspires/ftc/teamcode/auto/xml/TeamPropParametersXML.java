package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the information
// needed for our OpenCV methods to recognize a team prop during Autonomous.
public class TeamPropParametersXML {
    public static final String TAG = TeamPropParametersXML.class.getSimpleName();
    private static final String TEAM_PROP_FILE_NAME = "TeamPropParameters.xml";

    private final Document document;
    private final XPath xpath;

    public TeamPropParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + TEAM_PROP_FILE_NAME));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }
    }

    public TeamPropParameters getTeamPropParameters() throws XPathExpressionException {
        XPathExpression expr;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML team_prop_parameters");

        expr = xpath.compile("//team_prop_parameters");
        Node team_prop_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (team_prop_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//team_prop_parameters' not found");

        // Point to <red_channel_circles>
        Node red_channel_node = team_prop_parameters_node.getFirstChild();
        red_channel_node = XMLUtils.getNextElement(red_channel_node);
        if ((red_channel_node == null) || !red_channel_node.getNodeName().equals("red_channel_circles"))
            throw new AutonomousRobotException(TAG, "Element 'red_channel_circles' not found");

        // Point to <gray_parameters>
        Node gray_parameters_node = red_channel_node.getFirstChild();
        gray_parameters_node = XMLUtils.getNextElement(gray_parameters_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        VisionParameters.GrayParameters grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hough_circles_function_call_parameters>
        Node circles_node = gray_parameters_node.getNextSibling();
        circles_node = XMLUtils.getNextElement(circles_node);
        if ((circles_node == null) || !circles_node.getNodeName().equals("hough_circles_function_call_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hough_circles_function_call_parameters' not found");

        // Parse the HoughCircles parameter dp.
        Node dp_node = circles_node.getFirstChild();
        dp_node = XMLUtils.getNextElement(dp_node);
        if ((dp_node == null) || !dp_node.getNodeName().equals("dp") || dp_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'dp' not found");

        String dpText = dp_node.getTextContent();
        double dp;
        try {
            dp = Double.parseDouble(dpText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'dp'");
        }

        // Parse the HoughCircles parameter minDist.
        Node min_dist_node = dp_node.getNextSibling();
        min_dist_node = XMLUtils.getNextElement(min_dist_node);
        if ((min_dist_node == null) || !min_dist_node.getNodeName().equals("minDist") || min_dist_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'minDist' not found");

        String minDistText = min_dist_node.getTextContent();
        double minDist;
        try {
            minDist = Double.parseDouble(minDistText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'minDist'");
        }

        // Parse the HoughCircles parameter param1.
        Node param1_node = min_dist_node.getNextSibling();
        param1_node = XMLUtils.getNextElement(param1_node);
        if ((param1_node == null) || !param1_node.getNodeName().equals("param1") || param1_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'param1' not found");

        String param1Text = param1_node.getTextContent();
        double param1;
        try {
            param1 = Double.parseDouble(param1Text);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'param1'");
        }

        // Parse the HoughCircles parameter param2.
        Node param2_node = param1_node.getNextSibling();
        param2_node = XMLUtils.getNextElement(param2_node);
        if ((param2_node == null) || !param2_node.getNodeName().equals("param2") || param2_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'param2' not found");

        String param2Text = param2_node.getTextContent();
        double param2;
        try {
            param2 = Double.parseDouble(param2Text);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'param2'");
        }

        // Parse the HoughCircles parameter minRadius.
        Node min_radius_node = param2_node.getNextSibling();
        min_radius_node = XMLUtils.getNextElement(min_radius_node);
        if ((min_radius_node == null) || !min_radius_node.getNodeName().equals("minRadius") || min_radius_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'minRadius' not found");

        String minRadiusText = min_radius_node.getTextContent();
        int minRadius;
        try {
            minRadius = Integer.parseInt(minRadiusText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'minRadius'");
        }

        // Parse the HoughCircles parameter maxRadius.
        Node max_radius_node = min_radius_node.getNextSibling();
        max_radius_node = XMLUtils.getNextElement(max_radius_node);
        if ((max_radius_node == null) || !max_radius_node.getNodeName().equals("maxRadius") || max_radius_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'maxRadius' not found");

        String maxRadiusText = max_radius_node.getTextContent();
        int maxRadius;
        try {
            maxRadius = Integer.parseInt(maxRadiusText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'maxRadius'");
        }

        TeamPropParameters.HoughCirclesFunctionCallParameters houghCirclesFunctionCallParameters =
                new TeamPropParameters.HoughCirclesFunctionCallParameters(dp, minDist,
                        param1, param2, minRadius, maxRadius);

        // Parse the size criteria for the circles.
        Node criteria_node = circles_node.getNextSibling();
        criteria_node = XMLUtils.getNextElement(criteria_node);
        if (criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'criteria' not found");

        // Parse the <min_bounding_box_area> element.
        Node max_circles_node = criteria_node.getFirstChild();
        max_circles_node = XMLUtils.getNextElement(max_circles_node);
        if (max_circles_node == null || !max_circles_node.getNodeName().equals("max_circles") || max_circles_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'max_circles' not found or empty");

        String maxCirclesText = max_circles_node.getTextContent();
        int maxCircles;
        try {
            maxCircles = Integer.parseInt(maxCirclesText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'max_circles'");
        }

        return new TeamPropParameters(new TeamPropParameters.RedChannelCirclesParameters(grayParameters,
                houghCirclesFunctionCallParameters, maxCircles));
    }

}

