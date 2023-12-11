package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.common.xml.ImageXML;
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
        RobotLogCommon.c(TAG, "Parsing XML team_prop_parameters");

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

        // Point to <color_channel_pixel_count>
        Node pixel_count_node = circles_node.getNextSibling();
        pixel_count_node = XMLUtils.getNextElement(pixel_count_node);
        if ((pixel_count_node == null) || !pixel_count_node.getNodeName().equals("color_channel_pixel_count"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count' not found");

        // Point to <RED> for the red alliance parameters.
        Node red_pixel_count_node = pixel_count_node.getFirstChild();
        red_pixel_count_node = XMLUtils.getNextElement(red_pixel_count_node);
        if ((red_pixel_count_node == null) || !red_pixel_count_node.getNodeName().equals("RED"))
            throw new AutonomousRobotException(TAG, "Element 'RED' under 'color_channel_pixel_count' not found");

        // Point to <gray_parameters>
        Node red_pixel_count_gray_node = red_pixel_count_node.getFirstChild();
        red_pixel_count_gray_node = XMLUtils.getNextElement(red_pixel_count_gray_node);
        if ((red_pixel_count_gray_node == null) || !red_pixel_count_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/RED/gray_parameters' not found");

        VisionParameters.GrayParameters redPixelCountGrayParameters = ImageXML.parseGrayParameters(red_pixel_count_gray_node);

        // Point to the criteria for the red pixel count.
        Node red_pixel_count_criteria_node = red_pixel_count_gray_node.getNextSibling();
        red_pixel_count_criteria_node = XMLUtils.getNextElement(red_pixel_count_criteria_node);
        if (red_pixel_count_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/RED/criteria' not found");

        // Parse the <min_white_pixel_count> element.
        Node red_min_pixels_node = red_pixel_count_criteria_node.getFirstChild();
        red_min_pixels_node = XMLUtils.getNextElement(red_min_pixels_node);
        if (red_min_pixels_node == null || !red_min_pixels_node.getNodeName().equals("min_white_pixel_count") ||
                red_min_pixels_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/RED/criteria/min_white_pixel_count' not found or empty");

        String redMinPixelsText = red_min_pixels_node.getTextContent();
        int redMinPixelCount;
        try {
            redMinPixelCount = Integer.parseInt(redMinPixelsText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'color_channel_pixel_count/RED/criteria/min_white_pixel_count'");
        }

        // Point to <BLUE> for the blue alliance parameters.
        Node blue_pixel_count_node = red_pixel_count_node.getNextSibling();
        blue_pixel_count_node = XMLUtils.getNextElement(blue_pixel_count_node);
        if ((blue_pixel_count_node == null) || !blue_pixel_count_node.getNodeName().equals("BLUE"))
            throw new AutonomousRobotException(TAG, "Element 'BLUE' under 'color_channel_pixel_count' not found");

        // Point to <gray_parameters>
        Node blue_pixel_count_gray_node = blue_pixel_count_node.getFirstChild();
        blue_pixel_count_gray_node = XMLUtils.getNextElement(blue_pixel_count_gray_node);
        if ((blue_pixel_count_gray_node == null) || !blue_pixel_count_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/BLUE/gray_parameters' not found");

        VisionParameters.GrayParameters bluePixelCountGrayParameters = ImageXML.parseGrayParameters(blue_pixel_count_gray_node);

        // Point to the criteria for the blue pixel count.
        Node blue_pixel_count_criteria_node = blue_pixel_count_gray_node.getNextSibling();
        blue_pixel_count_criteria_node = XMLUtils.getNextElement(blue_pixel_count_criteria_node);
        if (blue_pixel_count_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/BLUE/criteria' not found");

        // Parse the <min_white_pixel_count> element.
        Node blue_min_pixels_node = blue_pixel_count_criteria_node.getFirstChild();
        blue_min_pixels_node = XMLUtils.getNextElement(blue_min_pixels_node);
        if (blue_min_pixels_node == null || !blue_min_pixels_node.getNodeName().equals("min_white_pixel_count") ||
                blue_min_pixels_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'color_channel_pixel_count/BLUE/criteria/min_white_pixel_count' not found or empty");

        String blueMinPixelsText = blue_min_pixels_node.getTextContent();
        int blueMinPixelCount;
        try {
            blueMinPixelCount = Integer.parseInt(blueMinPixelsText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'color_channel_pixel_count/BLUE/criteria/min_white_pixel_count'");
        }

        TeamPropParameters.ColorChannelPixelCountParameters colorChannelPixelCountParameters =
                new TeamPropParameters.ColorChannelPixelCountParameters(redPixelCountGrayParameters, redMinPixelCount,
                        bluePixelCountGrayParameters, blueMinPixelCount);

        // Point to <bright_spot>
        Node bright_spot_node = pixel_count_node.getNextSibling();
        bright_spot_node = XMLUtils.getNextElement(bright_spot_node);
        if ((bright_spot_node == null) || !bright_spot_node.getNodeName().equals("bright_spot"))
            throw new AutonomousRobotException(TAG, "Element 'bright_spot' not found");

        // Point to <RED> for the red alliance parameters.
        Node red_bright_node = bright_spot_node.getFirstChild();
        red_bright_node = XMLUtils.getNextElement(red_bright_node);
        if ((red_bright_node == null) || !red_bright_node.getNodeName().equals("RED"))
            throw new AutonomousRobotException(TAG, "Element 'RED' under 'bright_spot' not found");

        // Point to <gray_parameters>
        Node red_bright_gray_node = red_bright_node.getFirstChild();
        red_bright_gray_node = XMLUtils.getNextElement(red_bright_gray_node);
        if ((red_bright_gray_node == null) || !red_bright_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'bright_spot/RED/gray_parameters' not found");

        VisionParameters.GrayParameters redBrightSpotGrayParameters = ImageXML.parseGrayParameters(red_bright_gray_node);

        // Parse the <blur_kernel> element.
        Node red_bright_blur_kernel_node = red_bright_gray_node.getNextSibling();
        red_bright_blur_kernel_node = XMLUtils.getNextElement(red_bright_blur_kernel_node);
        if ((red_bright_blur_kernel_node == null) || !red_bright_blur_kernel_node.getNodeName().equals("blur_kernel") || red_bright_blur_kernel_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'bright_spot/RED/blur_kernel' not found or empty");

        String redBlurKernelText = red_bright_blur_kernel_node.getTextContent();
        double redBlurKernel;
        try {
            redBlurKernel = Double.parseDouble(redBlurKernelText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'bright_spot/RED/blur_kernel'");
        }

        // Point to <BLUE> for the blue alliance parameters.
        Node blue_bright_node = red_bright_node.getNextSibling();
        blue_bright_node = XMLUtils.getNextElement(blue_bright_node);
        if ((blue_bright_node == null) || !blue_bright_node.getNodeName().equals("BLUE"))
            throw new AutonomousRobotException(TAG, "Element 'BLUE' under 'bright_spot' not found");

        // Point to <gray_parameters>
        Node blue_bright_gray_node = blue_bright_node.getFirstChild();
        blue_bright_gray_node = XMLUtils.getNextElement(blue_bright_gray_node);
        if ((blue_bright_gray_node == null) || !blue_bright_gray_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'bright_spot/BLUE/gray_parameters' not found");

        VisionParameters.GrayParameters blueBrightSpotGrayParameters = ImageXML.parseGrayParameters(blue_bright_gray_node);

        // Parse the <blur_kernel> element.
        Node blue_bright_blur_kernel_node = blue_bright_gray_node.getNextSibling();
        blue_bright_blur_kernel_node = XMLUtils.getNextElement(blue_bright_blur_kernel_node);
        if ((blue_bright_blur_kernel_node == null) || !blue_bright_blur_kernel_node.getNodeName().equals("blur_kernel") || blue_bright_blur_kernel_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'bright_spot/BLUE/blur_kernel' not found or empty");

        String blueBlurKernelText = blue_bright_blur_kernel_node.getTextContent();
        double blueBlurKernel;
        try {
            blueBlurKernel = Double.parseDouble(blueBlurKernelText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'bright_spot/BLUE/blur_kernel'");
        }

        TeamPropParameters.BrightSpotParameters brightSpotParameters =
                new TeamPropParameters.BrightSpotParameters(redBrightSpotGrayParameters, redBlurKernel,
                        blueBrightSpotGrayParameters, blueBlurKernel);

        return new TeamPropParameters(colorChannelCirclesParameters, colorChannelPixelCountParameters, brightSpotParameters);
    }

}

