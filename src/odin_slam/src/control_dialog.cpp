#include "control_dialog.hpp"
#include <QVBoxLayout>
#include <QPushButton>

ControlDialog::ControlDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("control_dialog");
    resize(500, 300);

    QVBoxLayout *layout = new QVBoxLayout(this);
    m_btn_run_slam = new QPushButton("运行 SLAM", this);
    m_btn_stop_slam = new QPushButton("暂停 SLAM", this);
    m_btn_save_result = new QPushButton("保存结果", this);
    layout->addWidget(m_btn_run_slam);
    layout->addWidget(m_btn_stop_slam);
    layout->addWidget(m_btn_save_result);

    connect(m_btn_run_slam, &QPushButton::clicked, this, &ControlDialog::on_run_slam_clicked);
    connect(m_btn_stop_slam, &QPushButton::clicked, this, &ControlDialog::on_stop_slam_clicked);
    connect(m_btn_save_result, &QPushButton::clicked, this, &ControlDialog::on_save_result_clicked);
}

void ControlDialog::on_run_slam_clicked()
{
    m_btn_run_slam->setEnabled(false);
    m_btn_stop_slam->setEnabled(true);
    emit run_slam();
}

void ControlDialog::on_stop_slam_clicked()
{
    m_btn_run_slam->setEnabled(true);
    m_btn_stop_slam->setEnabled(false);
    emit stop_slam();
}

void ControlDialog::on_save_result_clicked()
{
    emit save_result();
}