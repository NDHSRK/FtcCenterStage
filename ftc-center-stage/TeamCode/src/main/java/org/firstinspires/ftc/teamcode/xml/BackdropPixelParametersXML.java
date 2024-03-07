package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

//!! WARNING: DO NOT copy the file/class of the same name from IJCenterStageVision
// and paste it here. This file supports changes to the XML for the configuration
// utility BackdropPixelViewer.

// Class whose job it is to read an XML file that contains all of the information
// needed for our OpenCV methods to recognize a pixel on the backdrop
// during Autonomous.
public class BackdropPixelParametersXML {
    public static final String TAG = BackdropPixelParametersXML.class.getSimpleName();
    private static final String BPP_FILE_NAME = "BackdropPixelParameters.xml";

    private final Document document;
    private final String xmlDirectory;
    private final String xmlFilePath;
    private final Node backdrop_pixel_gray_median_node;
    private final Node backdrop_pixel_gray_threshold_node;
    private final BackdropPixelParameters backdropPixelParameters;

    public BackdropPixelParametersXML(String pXMLDir) {
        Node backdrop_pixel_parameters_node;
        try {
            xmlDirectory = pXMLDir;
            xmlFilePath = pXMLDir + BPP_FILE_NAME;

            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(xmlFilePath));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            XPath xpath = xpathFactory.newXPath();

            // Point to the first node.
            XPathExpression expr = xpath.compile("//backdrop_pixel_parameters");
            backdrop_pixel_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
            if (backdrop_pixel_parameters_node == null)
                throw new AutonomousRobotException(TAG, "Element '//backdrop_pixel_parameters' not found");

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        } catch (XPathExpressionException xex) {
            throw new AutonomousRobotException(TAG, "XPath Exception " + xex.getMessage());
        }

        // Point to <gray_parameters>
        Node gray_node = backdrop_pixel_parameters_node.getFirstChild();
        Node gray_parameters_node = XMLUtils.getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        VisionParameters.GrayParameters grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Get access to the <median_target> and <threshold_low> elements under <gray_parameters>
        // for possible modification. NOTE: there is no possibility of a NPE here since the
        // <gray_parameters> element has already been parsed.
        Node local_backdrop_pixel_gray_median_node = gray_parameters_node.getFirstChild();
        backdrop_pixel_gray_median_node = XMLUtils.getNextElement(local_backdrop_pixel_gray_median_node);
        Node local_red_pixel_count_gray_threshold_node = backdrop_pixel_gray_median_node.getNextSibling();
        backdrop_pixel_gray_threshold_node = XMLUtils.getNextElement(local_red_pixel_count_gray_threshold_node);

        // Parse the criteria for the AprilTag and yellow pixel.
        Node criteria_node = gray_parameters_node.getNextSibling();
        criteria_node = XMLUtils.getNextElement(criteria_node);
        if (criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'criteria' not found");

        // Step down to the <april_tag_rectangle> element.
        Node april_tag_rectangle_node = criteria_node.getFirstChild();
        april_tag_rectangle_node = XMLUtils.getNextElement(april_tag_rectangle_node);
        if (april_tag_rectangle_node == null || !april_tag_rectangle_node.getNodeName().equals("april_tag_rectangle"))
            throw new AutonomousRobotException(TAG, "Element 'criteria/april_tag_rectangle' not found");

        BackdropPixelParameters.Criteria aprilTagCriteria = parseCriteria(april_tag_rectangle_node);

        // Parse the criteria for the yellow pixel bounding box.
        Node yellow_pixel_criteria_node = april_tag_rectangle_node.getNextSibling();
        yellow_pixel_criteria_node = XMLUtils.getNextElement(yellow_pixel_criteria_node);
        if (yellow_pixel_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'criteria/yellow_pixel' not found");

        BackdropPixelParameters.Criteria yellowPixelCriteria = parseCriteria(yellow_pixel_criteria_node);

        backdropPixelParameters = new BackdropPixelParameters(grayParameters, aprilTagCriteria, yellowPixelCriteria);
    }

    public BackdropPixelParameters getBackdropPixelParameters() {
        return backdropPixelParameters;
    }

    // Replaces the text values of the children of the <gray_parameters> element.
    public void setBackdropPixelGrayParameters(VisionParameters.GrayParameters pGrayParameters) {
        RobotLogCommon.c(TAG, "Setting the grayscale parameters for backdrop pixel recognition in backdropPixelParameters");
        RobotLogCommon.c(TAG, "Setting the grayscale median target to " + pGrayParameters.median_target);
        backdrop_pixel_gray_median_node.setTextContent(Integer.toString(pGrayParameters.median_target));

        RobotLogCommon.c(TAG, "Setting the grayscale threshold to " + pGrayParameters.threshold_low);
        backdrop_pixel_gray_threshold_node.setTextContent(Integer.toString(pGrayParameters.threshold_low));
    }

    public void writeBackdropPixelParametersFile() {
        XMLUtils.writeXMLFile(document, xmlFilePath, xmlDirectory + RobotConstants.XSLT_FILE_NAME);
    }

    private BackdropPixelParameters.Criteria parseCriteria(Node pCriteriaChildNode) {
        // Step down to the <min_area> element.
        Node min_area_node = pCriteriaChildNode.getFirstChild();
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

        // Parse the <min_aspect_ratio> element.
        Node min_ratio_node = max_area_node.getNextSibling();
        min_ratio_node = XMLUtils.getNextElement(min_ratio_node);
        if (min_ratio_node == null || !min_ratio_node.getNodeName().equals("min_aspect_ratio") ||
                min_ratio_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'min_aspect_ratio' not found or empty");

        String minAspectRatioText = min_ratio_node.getTextContent();
        double minRatio;
        try {
            minRatio = Double.parseDouble(minAspectRatioText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'min_aspect_ratio'");
        }

        // Parse the <max_aspect_ratio> element.
        Node max_ratio_node = min_ratio_node.getNextSibling();
        max_ratio_node = XMLUtils.getNextElement(max_ratio_node);
        if (max_ratio_node == null || !max_ratio_node.getNodeName().equals("max_aspect_ratio") ||
                max_ratio_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'max_aspect_ratio' not found or empty");

        String maxAspectRatioText = max_ratio_node.getTextContent();
        double maxRatio;
        try {
            maxRatio = Double.parseDouble(maxAspectRatioText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'max_aspect_ratio'");
        }

        return new BackdropPixelParameters.Criteria(minArea, maxArea, minRatio, maxRatio);
    }

}

