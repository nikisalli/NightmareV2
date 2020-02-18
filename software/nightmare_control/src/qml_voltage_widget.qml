import QtQuick 2.4
import SDK 1.0
import QtQuick.Layouts 1.1


Rectangle {
    id: root
    Layout.alignment: Layout.Center
    width: 200
    height: 200
    color: "#303030"
    property real actVal: 0

    Connections{
        target: batteryWidget
        onRandomValueChanged: root.actVal = batteryWidget.randomValue
    }

    Rectangle {
        Layout.alignment: Layout.Center
        width: 200
        height: 200
        color: "#303030"
        border.color: "#303030"
        border.width: 3
        Text {
            id: name
            text: "Battery Voltage"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.topMargin: 5
            font.pointSize: 13
            color: "#6affcd"
        }

        RadialBar {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            width: 150
            height: 150 - (0.000000000001)*actVal
            penStyle: Qt.RoundCap
            progressColor: "#6affcd"
            foregroundColor: "#252525"
            dialWidth: 12
            minValue: 0
            maxValue: 9
            value: actVal
            suffixText: "V"
            textFont {
                family: "Halvetica"
                italic: false
                pointSize: 18
            }
            textColor: "#00ffc1"
        }
    }
}