package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
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
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class RobotConfigXML {

    public static final String TAG = "RobotConfigXML";
    private static final String FILE_NAME = "RobotConfig.xml";

    // IntelliJ only
    /*
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */
    // End IntelliJ only

    private final Map<String, RobotXMLElement> robotElementCollection = new HashMap<>();
    private final EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcams
            = new EnumMap<>(RobotConstantsCenterStage.InternalWebcamId.class);

    public RobotConfigXML(String pWorkingDirectory) throws ParserConfigurationException, SAXException, IOException, XPathExpressionException {

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

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        String configFilename = pWorkingDirectory + FILE_NAME;
        Document document = dBuilder.parse(new File(configFilename));

        // Collect all of the elements from the XML file.
        Element robotConfigRoot = document.getDocumentElement();
        NodeList configChildren = robotConfigRoot.getChildNodes();
        Node oneConfigNode;

        // Iterate through the XML elements in the configuration
        // file and store each one into a Map whose key is the
        // tag name of the element.
        RobotXMLElement robotXMLElement;
        RobotXMLElement mappedRobotXMLElement;
        for (int i = 0; i < configChildren.getLength(); i++) {
            oneConfigNode = configChildren.item(i);

            if (oneConfigNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            robotXMLElement = new RobotXMLElement((Element) oneConfigNode);
            mappedRobotXMLElement = robotElementCollection.put(robotXMLElement.getRobotXMLElementName(), robotXMLElement);
            if (mappedRobotXMLElement != null)
                RobotLogCommon.d(TAG, "Duplicate element " + robotXMLElement.getRobotXMLElementName());

            if (robotXMLElement.getRobotXMLElementName().equals("VISION_PORTAL_WEBCAM")) {
                RobotLogCommon.c(TAG, "Parsing XML for VisionPortal webcam(s)");
                if (!configuredWebcams.isEmpty())
                    throw new AutonomousRobotException(TAG, "Duplicate VISION_PORTAL_WEBCAM element");

                parseWebcamConfiguration(oneConfigNode);
            }
        }

        RobotLogCommon.i(TAG, "In RobotConfigXML; opened and parsed the XML file");
    }

    public XPathAccess getPath(String pElementName) {
        if (pElementName == null)
            throw new AutonomousRobotException(TAG, "Null element name not allowed");

        RobotXMLElement mappedRobotXMLElement = robotElementCollection.get(pElementName);
        if (mappedRobotXMLElement == null)
            throw new AutonomousRobotException(TAG, "No such element in RobotConfig.xml: " + pElementName);

        return new XPathAccess(mappedRobotXMLElement);
    }

    // Returns an empty map if the webcam is configured out.
    public EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> getConfiguredWebcams() {
        return configuredWebcams;
    }

    // Parse a VISION_PORTAL_WEBCAM element into its own structure.
    private void parseWebcamConfiguration(Node pWebcamNode) {
      NamedNodeMap configuration_attributes = pWebcamNode.getAttributes();
        Node configured_node = configuration_attributes.getNamedItem("configured");
        if (configured_node == null || configured_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Attribute 'configured' is missing or empty");

        String configuredAttribute = configured_node.getTextContent();
        if (!(configuredAttribute.equals("yes") || configuredAttribute.equals("no")))
            throw new AutonomousRobotException(TAG, "Attribute 'configured' must be 'yes' or 'no'");

        if (configuredAttribute.equals("no"))
            return;

        // There must be at least one webcam in the configuration.
        Node webcam_set_node = pWebcamNode.getFirstChild();
        webcam_set_node = XMLUtils.getNextElement(webcam_set_node);
        if (webcam_set_node == null || !webcam_set_node.getNodeName().equals("webcam_set"))
            throw new AutonomousRobotException(TAG, "Element 'webcam_set' not found");

        // Process all of the children of the <webcam_set> element.
        NodeList webcam_set_elements = webcam_set_node.getChildNodes();
        if (webcam_set_elements == null)
            throw new AutonomousRobotException(TAG, "Missing 'webcam' elements");

        XMLUtils.processElements(webcam_set_elements, (each_webcam) -> {
            VisionPortalWebcamConfiguration.ConfiguredWebcam webcamData = parseWebcamData(each_webcam);

            RobotLogCommon.d(TAG, "Configuring webcam with serial number " + webcamData.serialNumber);

            // Make sure there are no duplicate webcam ids or serial numbers.
            Optional<RobotConstantsCenterStage.InternalWebcamId> duplicate = configuredWebcams.entrySet().stream()
                    .filter(e -> e.getValue().serialNumber.equals(webcamData.serialNumber) ||
                            e.getKey() == webcamData.internalWebcamId)
                    .map(Map.Entry::getKey)
                    .findFirst();

            if (duplicate.isPresent())
                throw new AutonomousRobotException(TAG, "Duplicate serial number or webcam id");

            configuredWebcams.put(webcamData.internalWebcamId, webcamData);
        });
    }

    private VisionPortalWebcamConfiguration.ConfiguredWebcam parseWebcamData(Node pWebcamNode) {
        // <id>
        Node id_node = pWebcamNode.getFirstChild();
        id_node = XMLUtils.getNextElement(id_node);
        if (id_node == null || !id_node.getNodeName().equals("internal_id") ||
                id_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'internal_id' not found");

        RobotConstantsCenterStage.InternalWebcamId webcamId =
                RobotConstantsCenterStage.InternalWebcamId.valueOf(id_node.getTextContent().toUpperCase());
        RobotLogCommon.c(TAG, "Webcam with internal id " + webcamId + " is in the configuration");

        // <serial_number>
        Node serial_number_node = id_node.getNextSibling();
        serial_number_node = XMLUtils.getNextElement(serial_number_node);
        if (serial_number_node == null || !serial_number_node.getNodeName().equals("serial_number") ||
                serial_number_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'serial_number' not found");

        String serial_number = serial_number_node.getTextContent();

        // <resolution_width>
        Node resolution_width_node = serial_number_node.getNextSibling();
        resolution_width_node = XMLUtils.getNextElement(resolution_width_node);
        if (resolution_width_node == null || !resolution_width_node.getNodeName().equals("resolution_width") ||
                resolution_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'resolution_width' not found");

        String widthString = resolution_width_node.getTextContent();
        int resolution_width;
        try {
            resolution_width = Integer.parseInt(widthString);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'resolution_width'");
        }

        // <resolution_height>
        Node resolution_height_node = resolution_width_node.getNextSibling();
        resolution_height_node = XMLUtils.getNextElement(resolution_height_node);
        if (resolution_height_node == null || !resolution_height_node.getNodeName().equals("resolution_height") ||
                resolution_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'resolution_height' not found");

        String heightString = resolution_height_node.getTextContent();
        int resolution_height;
        try {
            resolution_height = Integer.parseInt(heightString);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'resolution_height'");
        }

        // Parse the <processor_set> element.
        Node processor_set_node = resolution_height_node.getNextSibling();
        processor_set_node = XMLUtils.getNextElement(processor_set_node);
        if (processor_set_node == null || !processor_set_node.getNodeName().equals("processor_set") ||
                processor_set_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'processor_set' not found");

        // Process all of the children of the <processor_set> element.
        NodeList processor_set_elements = processor_set_node.getChildNodes();
        if (processor_set_elements == null)
            throw new AutonomousRobotException(TAG, "Missing 'processor' elements");

        ArrayList<RobotConstantsCenterStage.ProcessorIdentifier> processorIds = new ArrayList<>();
        XMLUtils.processElements(processor_set_elements, (processor_node) -> {
            if (processor_node == null || !processor_node.getNodeName().equals("processor") ||
                    processor_node.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'processor' not found");

            RobotConstantsCenterStage.ProcessorIdentifier processorId =
                    RobotConstantsCenterStage.ProcessorIdentifier.valueOf(processor_node.getTextContent().toUpperCase());
            processorIds.add(processorId);
        });

        // Parse the optional <webcam_calibration_for_apriltags> element
        VisionPortalWebcamConfiguration.CameraCalibration calibration = null;
        Node calibration_node = processor_set_node.getNextSibling();
        calibration_node = XMLUtils.getNextElement(calibration_node);
        if (calibration_node != null) {
            if (!calibration_node.getNodeName().equals("webcam_calibration_for_apriltags"))
                throw new AutonomousRobotException(TAG, "Element 'webcam_calibration_for_apriltags' not found");

            /*
               <focal_length_x>627.41948883</focal_length_x>
               <focal_length_y>627.419488832</focal_length_y>
               <optical_center_x>301.424062225</optical_center_x>
               <optical_center_y>234.042415697</optical_center_y>
             */
            Node fx_node = calibration_node.getFirstChild();
            fx_node = XMLUtils.getNextElement(fx_node);
            if (fx_node == null || !fx_node.getNodeName().equals("focal_length_x") ||
                    fx_node.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'focal_length_x' not found");

            String fxString = fx_node.getTextContent();
            double fx;
            try {
                fx = Double.parseDouble(fxString);
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'focal_length_x'");
            }

            Node fy_node = fx_node.getNextSibling();
            fy_node = XMLUtils.getNextElement(fy_node);
            if (fy_node == null || !fy_node.getNodeName().equals("focal_length_y") ||
                    fy_node.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'focal_length_y' not found");

            String fyString = fy_node.getTextContent();
            double fy;
            try {
                fy = Double.parseDouble(fyString);
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'focal_length_y'");
            }

            Node cx_node = fy_node.getNextSibling();
            cx_node = XMLUtils.getNextElement(cx_node);
            if (cx_node == null || !cx_node.getNodeName().equals("optical_center_x") ||
                    cx_node.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'optical_center_x' not found");

            String cxString = cx_node.getTextContent();
            double cx;
            try {
                cx = Double.parseDouble(cxString);
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'optical_center_x'");
            }

            Node cy_node = cx_node.getNextSibling();
            cy_node = XMLUtils.getNextElement(cy_node);
            if (cy_node == null || !cy_node.getNodeName().equals("optical_center_y") ||
                    cy_node.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'optical_center_y' not found");

            String cyString = cy_node.getTextContent();
            double cy;
            try {
                cy = Double.parseDouble(cyString);
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'optical_center_y'");
            }

            calibration = new VisionPortalWebcamConfiguration.CameraCalibration(fx, fy, cx, cy);
        }

        return new VisionPortalWebcamConfiguration.ConfiguredWebcam(webcamId,
                serial_number, resolution_width, resolution_height, processorIds,
                calibration);
    }

}
