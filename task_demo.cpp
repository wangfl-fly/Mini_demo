#include "cleandockctrl.h"
#include "utility/basic_actions.h"
namespace StationHandle{

BT::Blackboard::Ptr IWorkTask::bb_ptr_;
AvaCleanDockMsg IWorkTask::msg;
AvaMsgStationCleanSet IWorkTask::mop_msg;
void SweepStationContorl::halt(){
    setStatus(BT::NodeStatus::IDLE);
}

void SweepStationContorl::RegisterEvent(std::function<void(void*) > func,uint32_t const& ms,bool const& flag){
    vec_task_.push_back(std::make_tuple(func,ms,flag));
}
unsigned int SweepStationContorl::GetTaskNumber(){
    if(!vec_task_.empty()){
        return vec_task_.size();
    }else return 0;
}
WashStatus SweepStationContorl::GetRunState(){
    return wash_state_;
}
uint32_t SweepStationContorl::GetTaskId(){
    return taskId_;
}

void SweepStationContorl::SetRunState(WashStatus state){
    wash_state_ =  state;
    blackboard()->set("wash_status",wash_state_);
}

int64_t SweepStationContorl::GetTaskTimeNow()
{
    int64_t tick_cnt = 0;
    blackboard()->get("current_tick_cnt", tick_cnt);
    return tick_cnt * 20;
}

void SweepStationContorl::AddTaskTimer(const TestTaskEvent& event, uint32_t ms, bool repeat,void*pUser)
{
    std::lock_guard<std::mutex> locker(mutex_);

    int64_t timeoutPoint = GetTaskTimeNow();
    TestTaskId timerId = {timeoutPoint+ms, ++lastTaskId_};
    DEBUG("ADD WASH TASK : %d",lastTaskId_);
    auto timer = std::make_shared<MyTask>(event, ms, repeat,pUser);
    timer->SetTaskNextTimeout(timeoutPoint);
    task_manag_.insert(std::pair<TestTaskId,std::shared_ptr<MyTask>>(timerId, std::move(timer)));

}

int64_t SweepStationContorl::GetTickRemaining()
{
    std::lock_guard<std::mutex> locker(mutex_);
    if(task_manag_.empty())
        return -1;
    int64_t ms = task_manag_.begin()->first.first - GetTaskTimeNow();
    if(ms <= 0)
        return 0;

    return ms;
}

void SweepStationContorl::HandleTaskEvent()
{
    if(!task_manag_.empty())
    {
        std::lock_guard<std::mutex> locker(mutex_);
        int64_t timePoint = GetTaskTimeNow();
        if(!task_manag_.empty() && task_manag_.begin()->first.first<=timePoint)
        {
            task_manag_.begin()->second->eventCallback(task_manag_.begin()->second->_pUser);
            TestTaskId t = {task_manag_.begin()->second->GetTaskNextTimeout(), task_manag_.begin()->first.second};
            taskId_ = t.second;
            DEBUG("taskId_ is %d",taskId_);
            task_manag_.erase(task_manag_.begin());
        }
    }
}
//清洗流程初始化
void SweepStationContorl::Init(){
    isruning_ = false;
    iTickRemain_ = 0;
    task_num_ = 0;
    lastTaskId_ = 0;
    taskId_ = 0;
    if(!vec_task_.empty()){
        std::vector<std::tuple<std::function<void(void*) >,uint32_t,bool>>  vec_erase;
        vec_task_.swap(vec_erase);
    }
    if(!task_manag_.empty()){
        std::map<TestTaskId,std::shared_ptr<MyTask>>().swap(task_manag_);
    }
    TRACE("Wash task init...");
}

//清洗流程控制
BT::NodeStatus SweepStationContorl::tick()
{
    NodeStatus state = BT::NodeStatus::RUNNING;
    if(GetRunState() == WashStatus::WASH_IDLE){
        Init();
        int water_ml = 0;
        if (getParam<int>("work_mode", water_ml)== false) {
            WARN("Err Para  In This Func :SweepStationContorl !!!");
            water_ml = BT::convertFromString<int>(requiredNodeParameters().at("work_mode"));
        }
        if(water_ml == 100) wash_mode_ = WashMode::WASH_ONE;
        else if(water_ml == 200) wash_mode_ = WashMode::WASH_TWO;
        else if(water_ml == 300) wash_mode_ = WashMode::WASH_THREE;
        else if(water_ml == 101)  wash_mode_ = WashMode::WASH_FIRST;
        else if(water_ml == 102)  wash_mode_ = WashMode::WASH_LAST;
        else if(water_ml == 50)  wash_mode_ = WashMode::WASH_MIJIA;
        else wash_mode_ = WashMode::WASH_ONE;
        std::unique_ptr<IWorkTask> m_task = TaskFactory::CreateTask(wash_mode_);
        DEBUG("wash_mode is %d",wash_mode_);
        m_task->SetSweepCtrl(this);
        m_task->SetBbPtr(blackboard());
        m_task->AddTask();
        if(!isruning_){
            if(!vec_task_.empty()){
                for(auto const &task_test : vec_task_){
                    const TestTaskEvent m_func = std::get<0>(task_test);
                    const uint32_t m_ms   = std::get<1>(task_test);
                    const bool m_flag     = std::get<2>(task_test);
                    AddTaskTimer(m_func, m_ms, m_flag,(void*)0);
                }
            }
        }
        task_num_ = static_cast<uint32_t>(GetTaskNumber());
        DEBUG("my test : task_num_ is %d",task_num_);
        iTickRemain_ = GetTickRemaining();
        total_cnt = (iTickRemain_ / 20) < 1 ? 1 : (iTickRemain_ / 20);
        DEBUG("my test : total_cnt_ is & iTimeRemain_ is : %hd,%lld",total_cnt,iTickRemain_);
        now_cnt = 0;
        if(task_num_ > 0){
            isruning_ = true;
            SetRunState(WashStatus::WASH_RUNNING);
        }
    }
    blackboard()->get("wash_status",wash_state_);
    switch(wash_state_) {
        case WashStatus::WASH_RUNNING:
            if(task_num_ >= GetTaskId()){
                now_cnt++;
                if(now_cnt >= total_cnt){
                    now_cnt = 0;
                    HandleTaskEvent();
                    iTickRemain_ = GetTickRemaining();
                    total_cnt = iTickRemain_ / 20;
                    DEBUG("my test : iTimeRemain_ is : %lld",iTickRemain_);
                    DEBUG("my test : GetTaskId : %d",GetTaskId());
                    if(task_num_ == GetTaskId()) {
                        DEBUG("Washing END...");
                        SetRunState(WashStatus::WASH_DONE);
                    }
                }
                DEBUG("my test : now_cnt & total_cnt is : %d,%d",now_cnt,total_cnt);
            }
            break;
        case WashStatus::WASH_PAUSE:
            Init();
            state=BT::NodeStatus::SUCCESS;
            break;
        case WashStatus::WASH_DONE:
            Init();
            state=BT::NodeStatus::SUCCESS;
            break;
        case WashStatus::WASH_INTERRUPT:
            Init();
            state=BT::NodeStatus::FAILURE;
            break;
        default:
            break;
    }
    blackboard()->set("wash_status",wash_state_);
    return state;
}


void RegisterNodes(BehaviorTreeFactory& factory) {
    factory.registerNodeType<SweepStationContorl>("SweepStationContorl");
}


void IWorkTask::SetSweepCtrl(SweepStationContorl* sweep_station){
    sweep_station_ = sweep_station;
}

void IWorkTask::SetBbPtr(const BT::Blackboard::Ptr& bb){
       bb_ptr_ = bb;
}

void WorkTask1::AddTask(){
    if(sweep_station_){
        sweep_station_->RegisterEvent(WorkTask1::Task11, 1, false);
        sweep_station_->RegisterEvent(WorkTask1::Task12, 25000, false);
        sweep_station_->RegisterEvent(WorkTask1::Task13, 28000, false);
        sweep_station_->RegisterEvent(WorkTask1::Task114, 35000, false);
        sweep_station_->RegisterEvent(WorkTask1::Task14, 45000, false);
        sweep_station_->RegisterEvent(WorkTask1::Task15, 60000, false);
        if(sweep_station_->wash_mode_ == WashMode::WASH_ONE ||
           sweep_station_->wash_mode_ == WashMode::WASH_LAST){
            sweep_station_->RegisterEvent(WorkTask1::Task16, 125000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task17, 135000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task18, 143000, false);
        } else if(sweep_station_->wash_mode_ == WashMode::WASH_TWO){
            sweep_station_->RegisterEvent(WorkTask1::Task16, 115000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task17, 125000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task18, 133000, false);
        } else if(sweep_station_->wash_mode_ == WashMode::WASH_THREE ||
                  sweep_station_->wash_mode_ == WashMode::WASH_FIRST){
            sweep_station_->RegisterEvent(WorkTask1::Task16, 100000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task17, 110000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task18, 118000, false);
        } else {
            sweep_station_->RegisterEvent(WorkTask1::Task16, 125000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task17, 135000, false);
            sweep_station_->RegisterEvent(WorkTask1::Task18, 143000, false);
        }
    }
}

void WorkTask2::AddTask(){
    if(sweep_station_){
        sweep_station_->RegisterEvent(WorkTask2::Task21, 1, false);
        sweep_station_->RegisterEvent(WorkTask2::Task22, 15000, false);
        sweep_station_->RegisterEvent(WorkTask2::Task23, 25000, false);
        sweep_station_->RegisterEvent(WorkTask2::Task24, 32000, false);
        sweep_station_->RegisterEvent(WorkTask2::Task25, 40000, false);
        if(sweep_station_->wash_mode_ == WashMode::WASH_FIRST){
            sweep_station_->RegisterEvent(WorkTask2::Task26_9V, 45000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_7_and_8_35V, 60000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task29, 75000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_10, 85000, false);
        } else if(sweep_station_->wash_mode_ == WashMode::WASH_LAST){
            sweep_station_->RegisterEvent(WorkTask2::Task21, 45000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task22, 60000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task23, 70000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task24, 77000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task25, 85000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task26, 90000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_7_and_8_35V, 140000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task29, 155000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_10, 160000, false);
        }else if(sweep_station_->wash_mode_ == WashMode::WASH_TWO){
            sweep_station_->RegisterEvent(WorkTask2::Task26_14V, 45000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_7_and_8, 75000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task29, 83000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_10, 85000, false);
        }else if(sweep_station_->wash_mode_ == WashMode::WASH_THREE){
            sweep_station_->RegisterEvent(WorkTask2::Task26_10V, 45000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_7_and_8_35V, 75000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task29, 83000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_10, 85000, false);
        }
         else{
             if(sweep_station_->wash_mode_ == WashMode::WASH_ONE){
                sweep_station_->RegisterEvent(WorkTask2::Task26, 55000, false);
            }else{
                sweep_station_->RegisterEvent(WorkTask2::Task26_12V, 55000, false);
            }
            sweep_station_->RegisterEvent(WorkTask2::Task27, 105000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task28, 112000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task29, 125000, false);
            sweep_station_->RegisterEvent(WorkTask2::Task2_10, 130000, false);
        }

    }
}

std::unique_ptr<IWorkTask> TaskFactory::CreateTask(WashMode mode){
    switch(mode) {
        case WashMode::WASH_FIRST:
        case WashMode::WASH_ONE:
        case WashMode::WASH_TWO:
        case WashMode::WASH_THREE:
        case WashMode::WASH_LAST:
#ifndef CONFIG_CLOTHO_MOP_SET
            return std::unique_ptr<IWorkTask>(new WorkTask1());
            DEBUG("this is task1");
       // case WashMode::WASH_MIJIA:
#else
            DEBUG("this is task2");
            return std::unique_ptr<IWorkTask>(new WorkTask2());
#endif
        default:
            DEBUG("this is no 49 or 27");
            return std::unique_ptr<IWorkTask>(new WorkTask1());
        break;
    }
}

//具体任务的设定
void WorkTask1::Task11(void*){
    DEBUG("Task work tets1...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.rinsing_pump_vol = 70;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -85 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}

void WorkTask1::Task12(void*){
    DEBUG("Task work tets2...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.rinsing_pump_vol = 70;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = 85 / 2;
    bb_ptr_->set("mop_information", mop_msg);

}
void WorkTask1::Task13(void*){
    DEBUG("Task work tets3...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = 100;
    msg.rinsing_pump_vol = 70;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = 85 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}

void WorkTask1::Task114(void*){
    DEBUG("Task work tets4ss...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = 100;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = 85 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}

void WorkTask1::Task14(void*){
    DEBUG("Task work tets4...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = 85 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask1::Task15(void*){
    DEBUG("Task work tets5...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = 150 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask1::Task16(void*){
    DEBUG("Task work tets6...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = 100;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -85 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask1::Task17(void*){
    DEBUG("Task work tets7...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = 100;
    bb_ptr_->set("wash_ctrl_data", msg);
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask1::Task18(void*){
    DEBUG("Task work tets8...");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
    bb_ptr_->set("mop_information", mop_msg);
}

/*
* 2149清洗流程
*/
void WorkTask2::Task21(void*){
    DEBUG("Task work tets21... 打开清水电机,拖布内旋");
    msg = {0};
    mop_msg = {0};
    int value = 0;
    bb_ptr_->get("ElectricWaterSet",value);
    if(value == 1){
        msg.reserved_varible = 1;
        DEBUG("open electric water");
    }else{
        DEBUG("not open electric water");
    }
    msg.robot_work_state = 13;
    msg.rinsing_pump_vol = rinsing_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -35 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}

void WorkTask2::Task22(void*){
    DEBUG("Task work tets22... 打开抽污水电机");
    msg = {0};
    mop_msg = {0};
    int value = 0;
    bb_ptr_->get("ElectricWaterSet",value);
    if(value == 1){
        msg.reserved_varible = 1;
        DEBUG("open electric water");
    }else{
        DEBUG("not open electric water");
    }
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    msg.rinsing_pump_vol = rinsing_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -35 / 2;
    bb_ptr_->set("mop_information", mop_msg);

}
void WorkTask2::Task23(void*){
    DEBUG("Task work tets23... 关闭清水电机");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    msg.rinsing_pump_vol = rinsing_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -35 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task24(void*){
    DEBUG("Task work tets24... 关闭抽污水电机，拖布内旋 ");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -50 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task25(void*){
    DEBUG("Task work tets25... 拖布内旋 ");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -80 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task26(void*){
    DEBUG("Task work tets26... 拖布内旋15V ");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -150 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task26_12V(void*){
    DEBUG("Task work tets26... 拖布内旋12V ");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -120 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task26_14V(void*){
    DEBUG("Task work tets26... 拖布内旋14V ");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -140 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task26_10V(void*){
    DEBUG("Task work tets26... 拖布内旋10V ");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -100 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task26_9V(void*){
    DEBUG("Task work tets26... 拖布内旋10V ");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -90 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task27(void*){
    DEBUG("Task work tets27... 打开抽污水电机");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -35/ 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task28(void*){
    DEBUG("Task work tets28... 关闭拖布");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
    mop_msg.tuobu_speed      = -35 / 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task29(void*){
    DEBUG("Task work tets29... 关闭拖布");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task2_10(void*){
    DEBUG("Task work tets2_10... 关闭抽污水电机，打开风机");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task2_7_and_8(void*){
    DEBUG("Task work tets2_7_and_8... 拖布内旋5V,污水泵开启");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -50/ 2;
    bb_ptr_->set("mop_information", mop_msg);
}
void WorkTask2::Task2_7_and_8_35V(void*){
    DEBUG("Task work tets2_7_and_8... 拖布内旋35V,污水泵开启");
    msg = {0};
    mop_msg = {0};
    msg.robot_work_state = 13;
    msg.sewage_pump_vol  = sewage_pump_voltage;
    bb_ptr_->set("wash_ctrl_data", msg);
	mop_msg.tuobu_speed      = -35/ 2;
    bb_ptr_->set("mop_information", mop_msg);
}
}
