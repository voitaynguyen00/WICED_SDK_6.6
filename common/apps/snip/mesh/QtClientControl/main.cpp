#include "mainwindow.h"
#include <QApplication>

bool g_bUseBsa = false;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

#ifdef WICED_MESH_REPO
    g_bUseBsa = false;
#endif

#ifdef BSA
    g_bUseBsa = true;
#endif

    MainWindow w;
    w.show();

    return a.exec();
}
