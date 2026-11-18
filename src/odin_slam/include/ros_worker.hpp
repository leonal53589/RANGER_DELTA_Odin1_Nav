#pragma once

#include <QThread>

#include <atomic>
#include <memory>

class OdinSlamNode;

class RosWorker : public QThread
{
    Q_OBJECT
public:
    RosWorker(int argc, char **argv);
    virtual ~RosWorker() override;

    void run() override;
    OdinSlamNode* get_node();

signals:
    void NodeReady();
    void NodeFailed();

private:
    std::shared_ptr<OdinSlamNode> m_record_client;
    std::atomic<bool> m_run_flag = true;
    const int m_argc;
    char **m_argv;
};
