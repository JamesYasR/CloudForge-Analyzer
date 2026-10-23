#include "Scene/CloudScene.h"
#include "UndoRedoManager.h"
#include "Application/TaskRunner.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <atomic>
#include <iostream>
#include <stdexcept>

static int passed = 0, failed = 0;
static void check(bool value, const char* name) {
    value ? ++passed : ++failed;
    std::cout << (value ? "[PASS] " : "[FAIL] ") << name << std::endl;
}
template<class F> static bool waitFor(F ready) {
    QElapsedTimer timer; timer.start();
    while (!ready() && timer.elapsed() < 5000) { QCoreApplication::processEvents(); QThread::msleep(1); }
    return ready();
}

static void sceneTests()
{
    using namespace CloudScene;
    auto input = pcl::make_shared<Cloud>(); input->push_back({1,2,3}); input->push_back({4,5,6});
    Record source; source.id=1; source.name="source"; source.cloud=input;
    source.appearance.transform[3]=17; source.appearance.opacity=.4;
    source.appearance.pointSize=6; source.appearance.color={12,34,56};
    Snapshot before; before.generation=7; before.records.emplace(source.name, source);
    auto inside=pcl::make_shared<Cloud>(); inside->push_back(input->at(0));
    auto outside=pcl::make_shared<Cloud>(); outside->push_back(input->at(1));
    auto a=source,b=source; a.id=2; a.name="inside"; a.cloud=inside; b.id=3; b.name="outside"; b.cloud=outside;
    Transaction transaction(before);
    check(transaction.replace(1,{a,b}),"prepare two-output transaction");
    check(before.records.size()==1 && before.records.at("source").cloud==input,"preparation never mutates live input");
    const auto& edit=transaction.edit();
    check(edit.before.records.size()==1 && edit.after.records.size()==2,"all outputs are one edit");
    check(edit.after.records.at("inside").appearance==source.appearance,"prepared output preserves transform and display attributes");
    check(edit.after.records.at("inside").cloud==inside && input->size()==2,"history and edit share point buffers without copying input");
    { Transaction abandoned(before); abandoned.replace(1,{a}); }
    check(before.records.at("source").cloud==input,"abandoned transaction has no side effects");
    Transaction invalid(before);
    check(!invalid.replace(999,{a}) && !invalid.replace(1,{}),"reject missing source and no output");
    auto empty=a; empty.cloud=pcl::make_shared<Cloud>();
    check(!invalid.replace(1,{empty}),"reject empty retained output");
    auto collision=b; collision.name=a.name;
    check(!invalid.replace(1,{a,collision}) && invalid.edit().after.records.size()==1,"name collision cannot produce a partial edit");
    collision=b; collision.id=a.id;
    check(!invalid.replace(1,{a,collision}),"reject duplicate output identity");
    collision=a; collision.id=source.id;
    check(!invalid.replace(1,{collision}),"replacement cannot reuse source identity");

    UndoRedoManager history;
    check(!history.undo(before) && !history.redo(before),"empty history is a no-op");
    history.pushState(edit.description,edit.before,true);
    const auto restored=history.undo(edit.after);
    check(restored && restored->preserveCamera && restored->scene.records.size()==1,"one undo restores entire source scene");
    check(restored && restored->scene.records.at("source").id==1
          && restored->scene.records.at("source").appearance==source.appearance,"undo retains identity and all captured properties");
    const auto redone=history.redo(restored->scene);
    check(redone && redone->scene.records.count("inside") && redone->scene.records.count("outside"),"one redo restores both outputs");
    history.undo(edit.after);
    history.pushState("new edit",before);
    check(!history.canRedo(),"new committed edit invalidates redo");
    history.clear();
    for(int i=0;i<30;++i) history.pushState("shared",before);
    check(history.retainedBytes()==input->points.capacity()*sizeof(pcl::PointXYZ),"memory accounting counts shared buffers once");
    int count=0; while(history.undo(before)) ++count;
    check(count==UndoRedoManager::MAX_HISTORY,"history retains existing twenty-operation limit");
    history.clear(); check(!history.canUndo() && !history.canRedo() && history.retainedBytes()==0,"clear releases both history stacks");
}

static void taskTests()
{
    TaskRunner runner;
    bool done=false, gui=false;
    TaskRunner::Status status=TaskRunner::Status::Failed;
    auto context=std::make_shared<TaskContext>();
    std::atomic<bool> worker{false};
    check(runner.start(context,[&](const auto& task) {
        worker.store(QThread::currentThread()!=QCoreApplication::instance()->thread());
        task->report(1,2,"预览裁切");
    },[&](auto result) { status=result;gui=QThread::currentThread()==QCoreApplication::instance()->thread();done=true; }),"accept background task");
    check(!runner.start(context,[](const auto&){},[](auto){}),"reject concurrent submission");
    check(waitFor([&]{return done;}) && status==TaskRunner::Status::Success && worker && gui,"worker calculates and completion returns on GUI thread");
    check(context->current.load()==1 && context->total.load()==2,"progress belongs to independent task context");
    done=false;
    runner.start(std::make_shared<TaskContext>(),[](const auto&){throw std::runtime_error("injected");},[&](auto result){status=result;done=true;});
    check(waitFor([&]{return done;}) && status==TaskRunner::Status::Failed,"exception becomes failed outcome");
    done=false; context=std::make_shared<TaskContext>();context->cancelRequested.store(true);
    bool worked=false;
    runner.start(context,[&](const auto&){worked=true;},[&](auto result){status=result;done=true;});
    check(waitFor([&]{return done;}) && status==TaskRunner::Status::Cancelled && !worked,"pre-cancelled job never computes");
    done=false; context=std::make_shared<TaskContext>(); std::atomic<bool> computed{false},release{false};
    runner.start(context,[&](const auto&){computed=true;while(!release.load()) QThread::msleep(1);},[&](auto result){status=result;done=true;});
    check(waitFor([&]{return computed.load();}),"reach completion-race checkpoint");
    context->cancelRequested.store(true);release=true;
    check(waitFor([&]{return done;}) && status==TaskRunner::Status::Cancelled,"cancel wins over already-computed result");
    bool published=false; auto closing=std::make_shared<TaskContext>();
    {
        TaskRunner temporary;
        temporary.start(closing,[](const auto& task){while(!task->cancelRequested.load()) QThread::msleep(1);},[&](auto){published=true;});
        temporary.shutdown();
        QCoreApplication::processEvents();
    }
    QCoreApplication::processEvents();
    check(closing->cancelRequested.load() && !published,"shutdown cancels and waits without publishing into destroyed owner");
}

int main(int argc,char** argv)
{
    QCoreApplication app(argc,argv);
    sceneTests();taskTests();
    std::cout<<"[RESULT] pass="<<passed<<" fail="<<failed<<std::endl;
    return failed ? 1:0;
}
