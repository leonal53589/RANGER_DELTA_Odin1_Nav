#pragma once

#include <QDialog>

class QPushButton;

class ControlDialog : public QDialog
{
    Q_OBJECT
public:
    ControlDialog(QWidget* parent = nullptr);

public slots:
    void on_run_slam_clicked();
    void on_stop_slam_clicked();
    void on_save_result_clicked();

signals:
    void run_slam();
    void stop_slam();
    void save_result();

private:
    QPushButton* m_btn_run_slam = nullptr;
    QPushButton* m_btn_stop_slam = nullptr;
    QPushButton* m_btn_save_result = nullptr;
};