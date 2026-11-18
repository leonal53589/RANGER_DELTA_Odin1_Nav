#include <QApplication>
#include <QTimer>
#include <QMessageBox>

#include <rclcpp/rclcpp.hpp>

#include "ros_worker.hpp"
#include "control_dialog.hpp"
#include "odin_slam_node.hpp"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    ControlDialog dia;
    dia.show();

    RosWorker ros_worker(argc, argv);
    // 主事件循环启动后再开启 ROS 线程，确保监控机制正常
    QTimer::singleShot(0, &ros_worker, [&ros_worker]() { ros_worker.start(); });
    QObject::connect(
        &ros_worker,
        &RosWorker::NodeFailed,
        &dia, // 连接到当前线程
        [&dia]() {
            QMessageBox::critical(&dia, "Error", "Node failed.\n The app is stopping now.");
            QApplication::quit();
        }
    );

    QObject::connect(
        &ros_worker, &RosWorker::NodeReady,
        &dia, [&dia, &ros_worker]() {
            QObject::connect(&dia, &ControlDialog::run_slam, ros_worker.get_node(), &OdinSlamNode::run_slam);
            QObject::connect(&dia, &ControlDialog::stop_slam, ros_worker.get_node(), &OdinSlamNode::stop_slam);
            QObject::connect(&dia, &ControlDialog::save_result, ros_worker.get_node(), &OdinSlamNode::save_result);
        }
    );

    return app.exec();
}