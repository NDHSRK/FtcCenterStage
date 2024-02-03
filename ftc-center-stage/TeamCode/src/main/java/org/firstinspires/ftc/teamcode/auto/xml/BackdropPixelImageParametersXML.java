package org.firstinspires.ftc.teamcode.auto.xml;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.auto.vision.BackdropPixelParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.xml.ImageXML;
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

// Class whose job it is to read an XML file that contains information
// about the webcam image (dimensions and ROI) needed by the
// BackdropPixelViewer.
public class BackdropPixelImageParametersXML {
    public static final String TAG = BackdropPixelImageParametersXML.class.getSimpleName();
    private static final String BPIP_FILE_NAME = "BackdropPixelImageParameters.xml";

    private final Document document;
    private final String xmlFilePath;
    private final VisionParameters.ImageParameters backdropPixelImageParameters;

    public BackdropPixelImageParametersXML(String pXMLDir) {
        Node backdrop_pixel_image_parameters_node;
        try {
            xmlFilePath = pXMLDir + BPIP_FILE_NAME;

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
            XPathExpression expr = xpath.compile("//backdrop_pixel_image_parameters");
            backdrop_pixel_image_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
            if (backdrop_pixel_image_parameters_node == null)
                throw new AutonomousRobotException(TAG, "Element '//backdrop_pixel_image_parameters_node' not found");

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        } catch (XPathExpressionException xex) {
            throw new AutonomousRobotException(TAG, "XPath Exception " + xex.getMessage());
        }

        // Point to <image_parameters>
        Node image_parameters_node = backdrop_pixel_image_parameters_node.getFirstChild();
        image_parameters_node = XMLUtils.getNextElement(image_parameters_node);
        if ((image_parameters_node == null) || !image_parameters_node.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'image_parameters' not found");

        backdropPixelImageParameters = ImageXML.parseImageParameters(image_parameters_node);

    }

    public VisionParameters.ImageParameters getBackdropPixelImageParameters() {
        return backdropPixelImageParameters;
    }

}

