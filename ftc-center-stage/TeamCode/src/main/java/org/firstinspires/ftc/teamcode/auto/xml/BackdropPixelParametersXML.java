package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.auto.vision.BackdropPixelParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
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
// needed for our OpenCV methods to recognize a pixel on the backdrop
// during Autonomous.
public class BackdropPixelParametersXML {
    public static final String TAG = BackdropPixelParametersXML.class.getSimpleName();
    private static final String BDP_FILE_NAME = "BackdropPixelParameters.xml";

    private final Document document;
    private final XPath xpath;

    public BackdropPixelParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + BDP_FILE_NAME));
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

    public BackdropPixelParameters getBackdropPixelParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML backdrop_pixel_parameters");

        expr = xpath.compile("//backdrop_pixel_parameters");
        Node backdrop_pixel_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (backdrop_pixel_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//backdrop_pixel_parameters' not found");

        // Point to <gray_parameters>
        Node gray_node = backdrop_pixel_parameters_node.getFirstChild();
        Node gray_parameters_node = XMLUtils.getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Parse the size criteria for the AprilTag bounding box.
        Node criteria_node = gray_parameters_node.getNextSibling();
        criteria_node = XMLUtils.getNextElement(criteria_node);
        if (criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'criteria' not found");

        Node april_tag_criteria_node = criteria_node.getFirstChild();
        april_tag_criteria_node = XMLUtils.getNextElement(april_tag_criteria_node);
        if (april_tag_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'april_tag/criteria' not found");

        BackdropPixelParameters.BoundingBoxCriteria aprilTagCriteria = parseCriteria(april_tag_criteria_node);

        // Parse the criteria for the yellow pixel bounding box.
        Node yellow_pixel_criteria_node = april_tag_criteria_node.getNextSibling();
        yellow_pixel_criteria_node = XMLUtils.getNextElement(yellow_pixel_criteria_node);
        if (yellow_pixel_criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'yellow_pixel/criteria' not found");

        BackdropPixelParameters.BoundingBoxCriteria yellowPixelCriteria = parseCriteria(april_tag_criteria_node);

        return new BackdropPixelParameters(grayParameters, aprilTagCriteria, yellowPixelCriteria);
    }

    private BackdropPixelParameters.BoundingBoxCriteria parseCriteria(Node pCriteriaChildNode) {
        // Parse the <min_bounding_box_area> element.
        Node min_area_node = pCriteriaChildNode.getFirstChild();
        min_area_node = XMLUtils.getNextElement(min_area_node);
        if (min_area_node == null || !min_area_node.getNodeName().equals("min_bounding_box_area") ||
                min_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'min_bounding_box_area' not found or empty");

        String minAreaText = min_area_node.getTextContent();
        double minArea;
        try {
            minArea = Double.parseDouble(minAreaText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'min_bounding_box_area'");
        }

        // Parse the <max_bounding_box_area> element.
        Node max_area_node = min_area_node.getNextSibling();
        max_area_node = XMLUtils.getNextElement(max_area_node);
        if (max_area_node == null || !max_area_node.getNodeName().equals("max_bounding_box_area") ||
                max_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'max_bounding_box_area' not found or empty");

        String maxAreaText = max_area_node.getTextContent();
        double maxArea;
        try {
            maxArea = Double.parseDouble(maxAreaText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'max_bounding_box_area'");
        }

        return new BackdropPixelParameters.BoundingBoxCriteria(minArea, maxArea);
    }

}

