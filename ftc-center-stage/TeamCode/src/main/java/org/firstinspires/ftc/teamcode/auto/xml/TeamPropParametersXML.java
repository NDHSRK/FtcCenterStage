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

        // Point to <color_channel_circles>
        Node circles_node = team_prop_parameters_node.getFirstChild();
        circles_node = XMLUtils.getNextElement(circles_node);
        if ((circles_node == null) || !circles_node.getNodeName().equals("color_channel_circles"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_circles' not found");

        // Point to <gray_parameters>
        Node gray_parameters_node = circles_node.getFirstChild();
        gray_parameters_node = XMLUtils.getNextElement(gray_parameters_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' under 'color_channel_circles' not found");

        VisionParameters.GrayParameters grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hough_circles_function_call_parameters>
        Node hough_circles_parameters_node = gray_parameters_node.getNextSibling();
        hough_circles_parameters_node = XMLUtils.getNextElement(hough_circles_parameters_node);
        if ((hough_circles_parameters_node == null) || !hough_circles_parameters_node.getNodeName().equals("hough_circles_function_call_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hough_circles_function_call_parameters' not found");

        // Parse the HoughCircles parameter dp.
        Node dp_node = hough_circles_parameters_node.getFirstChild();
        dp_node = XMLUtils.getNextElement(dp_node);
        if ((dp_node == null) || !dp_node.getNodeName().equals("dp") || dp_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'dp' not found or empty");

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
            throw new AutonomousRobotException(TAG, "Element 'minDist' not found or empty");

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
            throw new AutonomousRobotException(TAG, "Element 'param1' not found or empty");

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
            throw new AutonomousRobotException(TAG, "Element 'param2' not found or empty");

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
            throw new AutonomousRobotException(TAG, "Element 'minRadius' not found or empty");

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
            throw new AutonomousRobotException(TAG, "Element 'maxRadius' not found or empty");

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
        Node criteria_node = hough_circles_parameters_node.getNextSibling();
        criteria_node = XMLUtils.getNextElement(criteria_node);
        if (criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'criteria' under 'color_channel_circles' not found");

        // Parse the <max_circles> element.
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

        TeamPropParameters.ColorChannelCirclesParameters colorChannelCirclesParameters =
                new TeamPropParameters.ColorChannelCirclesParameters(grayParameters,
                        houghCirclesFunctionCallParameters, maxCircles);

        // Point to <color_channel_features>
        Node features_node = circles_node.getNextSibling();
        features_node = XMLUtils.getNextElement(features_node);
        if ((features_node == null) || !features_node.getNodeName().equals("color_channel_features"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_features' not found");

        // Point to <gray_parameters>
        Node features_gray_node = features_node.getFirstChild();
        features_gray_node = XMLUtils.getNextElement(features_gray_node);
        if ((features_gray_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' under 'color_channel_features' not found");

        VisionParameters.GrayParameters featuresGrayParameters = ImageXML.parseGrayParameters(features_gray_node);

        // Point to <max_corners>
        Node max_corners_node = features_gray_node.getNextSibling();
        max_corners_node = XMLUtils.getNextElement(max_corners_node);
        if ((max_corners_node == null) || !max_corners_node.getNodeName().equals("max_corners") || max_corners_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'max_corners' not found or empty");

        String maxCornersText = max_corners_node.getTextContent();
        int maxCorners;
        try {
            maxCorners = Integer.parseInt(maxCornersText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'max_corners'");
        }

        // Point to <quality_level>
        Node quality_level_node = max_corners_node.getNextSibling();
        quality_level_node = XMLUtils.getNextElement(quality_level_node);
        if ((quality_level_node == null) || !quality_level_node.getNodeName().equals("quality_level") || quality_level_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'quality_level' not found or empty");

        String qualityLevelText = quality_level_node.getTextContent();
        double qualityLevel;
        try {
            qualityLevel = Double.parseDouble(qualityLevelText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'quality_level'");
        }

        TeamPropParameters.ColorChannelFeaturesParameters colorChannelFeaturesParameters =
                new TeamPropParameters.ColorChannelFeaturesParameters(featuresGrayParameters, maxCorners, qualityLevel);

        // Point to <color_channel_contours>
        Node contours_node = features_node.getNextSibling();
        contours_node = XMLUtils.getNextElement(contours_node);
        if ((contours_node == null) || !contours_node.getNodeName().equals("color_channel_contours"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_contours' not found");

        // Point to <gray_parameters>
        Node contours_gray_node = contours_node.getFirstChild();
        contours_gray_node = XMLUtils.getNextElement(contours_gray_node);
        if ((contours_gray_node == null) || !contours_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' under 'color_channel_contours' not found");

        VisionParameters.GrayParameters contoursGrayParameters = ImageXML.parseGrayParameters(contours_gray_node);

        // Point to the size criteria for the contours.
        Node contours_criteria_node = contours_gray_node.getNextSibling();
        contours_criteria_node = XMLUtils.getNextElement(contours_criteria_node);
        if (contours_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'criteria' under 'color_channel_contours' not found");

        // Parse the <min_area> element.
        Node min_area_node = contours_criteria_node.getFirstChild();
        min_area_node = XMLUtils.getNextElement(min_area_node);
        if (min_area_node == null || !min_area_node.getNodeName().equals("min_area") ||
                min_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'min_area' not found or empty");

        String minAreaText = min_area_node.getTextContent();
        double minArea;
        try {
            minArea = Double.parseDouble(minAreaText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'min_area'");
        }

        // Parse the <max_area> element.
        Node max_area_node = min_area_node.getNextSibling();
        max_area_node = XMLUtils.getNextElement(max_area_node);
        if (max_area_node == null || !max_area_node.getNodeName().equals("max_area") ||
                max_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'max_area' not found or empty");

        String maxAreaText = max_area_node.getTextContent();
        double maxArea;
        try {
            maxArea = Double.parseDouble(maxAreaText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'max_area'");
        }

        TeamPropParameters.ColorChannelContoursParameters colorChannelContoursParameters =
                new TeamPropParameters.ColorChannelContoursParameters(contoursGrayParameters,
                        minArea, maxArea);

        // Point to <bright_spot>
        Node bright_spot_node = contours_node.getNextSibling();
        bright_spot_node = XMLUtils.getNextElement(bright_spot_node);
        if ((bright_spot_node == null) || !bright_spot_node.getNodeName().equals("bright_spot"))
            throw new AutonomousRobotException(TAG, "Element 'bright_spot' not found");

        // Point to <gray_parameters>
        Node bright_spot_gray_node = bright_spot_node.getFirstChild();
        bright_spot_gray_node = XMLUtils.getNextElement(bright_spot_gray_node);
        if ((bright_spot_gray_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' under 'bright_spot' not found");

        VisionParameters.GrayParameters brightSpotGrayParameters = ImageXML.parseGrayParameters(bright_spot_gray_node);

        // Parse the <blur_kernel> element.
        Node blur_kernel_node = bright_spot_gray_node.getNextSibling();
        blur_kernel_node = XMLUtils.getNextElement(blur_kernel_node);
        if ((blur_kernel_node == null) || !blur_kernel_node.getNodeName().equals("blur_kernel") || blur_kernel_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'blur_kernel' not found or empty");

        String blurKernelText = blur_kernel_node.getTextContent();
        double blurKernel;
        try {
            blurKernel = Double.parseDouble(blurKernelText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'blur_kernel'");
        }

        TeamPropParameters.BrightSpotParameters brightSpotParameters =
                new TeamPropParameters.BrightSpotParameters(brightSpotGrayParameters, blurKernel);

        return new TeamPropParameters(colorChannelCirclesParameters, colorChannelFeaturesParameters,
                colorChannelContoursParameters, brightSpotParameters);
    }

}

