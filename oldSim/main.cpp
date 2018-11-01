#include <stdio.h>

#include "main_window.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    //GamepadMonitor monitor;
    printf("[Cheetah Control] Starting qt GUI...\n");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
