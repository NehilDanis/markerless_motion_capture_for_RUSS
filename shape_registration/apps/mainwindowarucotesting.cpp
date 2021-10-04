#include "shape_registration/apps/mainwindowarucotesting.h"
#include "ui_mainwindowarucotesting.h"

MainWindowArucoTesting::MainWindowArucoTesting(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindowArucoTesting)
{
  ui->setupUi(this);
}

MainWindowArucoTesting::~MainWindowArucoTesting()
{
  delete ui;
}

void MainWindowArucoTesting::on_pushButton_clicked()
{

}
