#ifndef STATIONCTRL_H
#define STATIONCTRL_H
#define sewage_pump_voltage 70
#define rinsing_pump_voltage 65

#include "behaviortree_cpp/bt_factory.h"
#include "actions/robot_common.h"
using namespace BT;
namespace StationHandle
{
//清洗模式
enum class WashMode{
    WASH_FIRST = 0,
    WASH_ONE,
    WASH_TWO,
    WASH_THREE,
    WASH_MIJIA, //P2149
    WASH_LAST,
};
typedef std::pair<int64_t, uint32_t> TestTaskId;
typedef std::function<void(void*)> TestTaskEvent;
class SweepStationContorl : public BT::ActionNodeBase
{
public:
    SweepStationContorl(const std::string& name, const BT::NodeParameters& params)
        : BT::ActionNodeBase(name, params){
            wash_state_ = WashStatus::WASH_IDLE;
            task_num_ = 0;
        }

    virtual BT::NodeStatus tick()override;
    virtual void halt() override;
    void Init();
    static const BT::NodeParameters& requiredNodeParameters()
    {
        static BT::NodeParameters params = {
                                                {"work_mode","100"}
                                                };
        return params;
    }
    WashStatus GetRunState();
    uint32_t GetTaskId();
    void SetRunState(WashStatus state);
    unsigned int GetTaskNumber(void);
    int64_t GetTaskTimeNow(void);
    int64_t GetTickRemaining(void);
    void HandleTaskEvent(void);
    void AddTaskTimer(const TestTaskEvent& event, uint32_t ms, bool repeat,void*pUser);
    void RegisterEvent(std::function<void(void*) > func,uint32_t const& ms,bool const& flag);
private:
    struct MyTask
    {
        MyTask(const TestTaskEvent& event, uint32_t ms, bool repeat,void* pUser)
            : eventCallback(event)
            , _interval(ms)
            , _isRepeat(repeat)
            ,_pUser(pUser)
        {
            if (_interval == 0)
                _interval = 1;
        }
        MyTask() { }
        void SetTaskNextTimeout(int64_t currentTimePoint){
            _nextTimeout = currentTimePoint + _interval;
        }
        int64_t GetTaskNextTimeout() const {
            return _nextTimeout;
        }
        TestTaskEvent eventCallback = [](void*){};
        uint32_t _interval = 0;
        bool _isRepeat = false;
        int64_t _nextTimeout = 0;
        void * _pUser;
    };
    WashStatus wash_state_;
    std::mutex mutex_;
    bool isruning_ = false;
    int64_t iTickRemain_ = 0;
    uint32_t task_num_;
    uint32_t lastTaskId_ = 0;
    int total_cnt;
    int now_cnt;
    uint32_t taskId_ = 0;
    std::vector<std::tuple<std::function<void(void*) >,uint32_t,bool> >  vec_task_;
    std::map<TestTaskId,std::shared_ptr<MyTask>>  task_manag_;
public:
    WashMode wash_mode_;
};

void RegisterNodes(BehaviorTreeFactory& factory);

/*
* 清洗任务
*/
class IWorkTask{
public:
    IWorkTask(){}
	virtual ~IWorkTask(){}
	virtual void AddTask() = 0;
    void SetBbPtr(const BT::Blackboard::Ptr& bb);
    void SetSweepCtrl(StationHandle::SweepStationContorl* sweep_station);
protected:
    SweepStationContorl* sweep_station_;
    static BT::Blackboard::Ptr bb_ptr_;
    static AvaCleanDockMsg msg;
    static AvaMsgStationCleanSet mop_msg;
};

class WorkTask1 : public IWorkTask{
public:
    WorkTask1(){}
	virtual ~WorkTask1(){}
    virtual void AddTask();
    static void Task11(void*);
    static void Task12(void*);
    static void Task13(void*);
    static void Task14(void*);
    static void Task15(void*);
    static void Task16(void*);
    static void Task17(void*);
    static void Task18(void*);
    static void Task19(void*);
    static void Task114(void*);
    friend class SweepStationContorl;
};

class WorkTask2 : public IWorkTask{
public:
    WorkTask2(){}
	virtual ~WorkTask2(){}
    virtual void AddTask();
    static void Task21(void*);
    static void Task22(void*);
    static void Task23(void*);
    static void Task24(void*);
    static void Task25(void*);
    static void Task26(void*);
    static void Task27(void*);
    static void Task28(void*);
    static void Task29(void*);
    static void Task2_10(void*);
    static void Task26_12V(void*);
    static void Task26_10V(void*);
    static void Task26_14V(void*);
    static void  Task26_9V(void*);
    static void Task2_7_and_8(void*);
    static void Task2_7_and_8_35V(void*);
    friend class SweepStationContorl;
};
class TaskFactory{
public:
	TaskFactory(){}
	virtual ~TaskFactory(){}
	static std::unique_ptr<IWorkTask> CreateTask(WashMode mode);
};

}
#endif // STATIONCTRL_H
