#ifndef MAINWINDOWARUCOTESTING_H
#define MAINWINDOWARUCOTESTING_H

#include <QMainWindow>

namespace Ui {
class MainWindowArucoTesting;
}

class MainWindowArucoTesting : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindowArucoTesting(QWidget *parent = nullptr);
  ~MainWindowArucoTesting();
  on_pushButton_clicked();

private:
  Ui::MainWindowArucoTesting *ui;
};

#endif // MAINWINDOWARUCOTESTING_H
