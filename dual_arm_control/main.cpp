#include <QApplication>
#include "ui/main_window.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("DualArmControl"));
    app.setApplicationVersion(QStringLiteral("1.0.0"));

    dac::MainWindow w;
    w.show();

    return app.exec();
}
