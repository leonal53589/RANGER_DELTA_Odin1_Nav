#include <exception>
#include <rclcpp/utilities.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <QEventLoop>
#include <QApplication>

#include "ros_worker.hpp"
#include "odin_slam_node.hpp"

RosWorker::RosWorker(int argc, char **argv)
    : m_argc(argc), m_argv(argv)
{
}

RosWorker::~RosWorker()
{
    m_run_flag = false;
    wait();
}

void RosWorker::run()
{
    rclcpp::init(
        m_argc, m_argv,
        rclcpp::InitOptions(),
        rclcpp::SignalHandlerOptions::None
    ); 

    try
    {
        m_record_client = std::make_shared<OdinSlamNode>();
    }
    catch (const std::exception &e)
    {
        emit NodeFailed();
        rclcpp::shutdown();
        return;
    }

    emit NodeReady();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(m_record_client);

    QEventLoop loop;
    while (m_run_flag)
    {
        executor.spin_some();
        loop.processEvents();
        msleep(5);
    }

    rclcpp::shutdown();
}

OdinSlamNode* RosWorker::get_node()
{
    return m_record_client.get();
}
