package org.firstinspires.ftc.ftcdevcommon.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.Consumer;

import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import javax.xml.transform.stream.StreamSource;

public class XMLUtils {

    private static final String TAG = XMLUtils.class.getSimpleName();

    public static void processElements(NodeList pNodeList, Consumer<Element> pProcessElement) {
        Node oneNode;
        for (int i = 0; i < pNodeList.getLength(); i++) {
            oneNode = pNodeList.item(i);

            if (oneNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            pProcessElement.accept((Element) oneNode);
        }
    }

    public static Node getNextElement(Node pNode) {
        Node nd = pNode;
        while (nd != null) {
            if (nd.getNodeType() == Node.ELEMENT_NODE) {
                return nd;
            }
            nd = nd.getNextSibling();
        }
        return null;
    }

    // Based on a combination of --
    // https://mkyong.com/java/how-to-modify-xml-file-in-java-dom-parser/#download-source-code
    // https://www.baeldung.com/java-pretty-print-xml
    // using the XSLT method to avoid extra blank lines in the output.
    public static void writeXMLFile(Document pDocument, String pDocumentFilePath, String pXSLTFilePath) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(pDocumentFilePath))) {
            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            //##!! Fails in Android "not supported" transformerFactory.setAttribute("indent-number", 4);
            Transformer transformer = transformerFactory.newTransformer(new StreamSource(new File(pXSLTFilePath)));
            transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
            transformer.setOutputProperty(OutputKeys.OMIT_XML_DECLARATION, "yes");
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");
            // From http://www.java2s.com/example/android/xml/indent-xml-content.html
            transformer.setOutputProperty(
                    "{http://xml.apache.org/xslt}indent-amount", "4"); //##!! Android only
            transformer.transform(new DOMSource(pDocument), new StreamResult(writer));
        } catch (IOException | TransformerException e) {
            throw new AutonomousRobotException(TAG, e.getMessage());
        }
    }
}
