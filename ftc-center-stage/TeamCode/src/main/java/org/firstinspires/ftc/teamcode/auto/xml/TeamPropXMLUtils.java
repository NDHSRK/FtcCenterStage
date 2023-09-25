package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class TeamPropXMLUtils {

    private static final String TAG = TeamPropXMLUtils.class.getSimpleName();

    // Get the team prop location actions associated with an OpMode.
    // The key of the return map is the team prop location.
    public static EnumMap<RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> getTeamPropLocationActions(Node pTeamPropLocationChoiceNode) {
        EnumMap< RobotConstantsCenterStage.TeamPropLocation, List<RobotXMLElement>> teamPropLocationActions = new EnumMap<>(RobotConstantsCenterStage.TeamPropLocation.class);
        List<RobotXMLElement> actions;

        NodeList teamPropLocationChoiceChildren = pTeamPropLocationChoiceNode.getChildNodes();
        if (teamPropLocationChoiceChildren.getLength() == 0)
            throw new AutonomousRobotException(TAG, "Missing elements under TEAM_PROP_LOCATION_CHOICE");

        Node teamPropLocationNode;
        int teamPropLocationCount = 0;
        for (int i = 0; i < teamPropLocationChoiceChildren.getLength(); i++) {
            teamPropLocationNode = teamPropLocationChoiceChildren.item(i);

            if (teamPropLocationNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            teamPropLocationCount++;
            RobotConstantsCenterStage.TeamPropLocation teamPropLocation = RobotConstantsCenterStage.TeamPropLocation.valueOf(teamPropLocationNode.getNodeName());
            actions = collectTeamPropLocationActions(teamPropLocationNode.getChildNodes());
            if (teamPropLocationActions.containsKey(teamPropLocation))
                throw new AutonomousRobotException(TAG, "Duplicate key in team prop location map");

            teamPropLocationActions.put(teamPropLocation, actions);
        }

        if (teamPropLocationCount != 3)
            throw new AutonomousRobotException(TAG, "Missing one or more location elements");

        return teamPropLocationActions;
    }

    // Iterate through the children of the team prop location node
    // and collect the elements. Note: a team prop location element with no
    // children is valid.
    private static List<RobotXMLElement> collectTeamPropLocationActions(NodeList pNodeList) {

        List<RobotXMLElement> actions = new ArrayList<>();
        Node oneActionNode;
        RobotXMLElement actionXMLElement;

        for (int i = 0; i < pNodeList.getLength(); i++) {
            oneActionNode = pNodeList.item(i);

            if (oneActionNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            actionXMLElement = new RobotXMLElement((Element) oneActionNode);
            actions.add(actionXMLElement);
        }

        return actions;
    }
}
