
#include <mainwindow.h>
#include <QApplication>
#include <iostream>

int main(int argc, char** argv) {

  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  std::cout<< "Exiting" << std::endl;
  return 0;
}
