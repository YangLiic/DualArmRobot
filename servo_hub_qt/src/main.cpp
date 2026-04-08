#include "app/MainWindow.h"
#include "communication/CommunicationManager.h"
#include "models/Types.h"
#include "services/MotorService.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("servo_hub_qt"));
    app.setOrganizationName(QStringLiteral("DualArmRobot"));

    registerCommonMetaTypes();

    CommunicationManager communicationManager;
    MotorService motorService(&communicationManager);
    MainWindow window(&motorService);
    window.show();

    return app.exec();
}
