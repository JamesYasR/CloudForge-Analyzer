// 只验证共享点云选择窗口，不运行各测量、拟合或滤波算法。
#include "CloudForgeAnalyzer.h"
#include "Basic/ViewportCloudSelector.h"
#include "Dialog/ChoseCloudDialog.h"
#include "Dialog/RmCloudDialog.h"
#include "Dialog/SaveCloudDialog.h"
#include <QElapsedTimer>
#include <QEventLoop>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QThread>
#include <vtkCamera.h>
#include <vtkPropCollection.h>
#include <iostream>

static int failures=0;
static void check(bool ok,const char* name) {
    std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<std::endl;
    if(!ok) ++failures;
}
static void pump(int ms=25) {
    QElapsedTimer timer;timer.start();
    do {QApplication::processEvents();QThread::msleep(1);} while(timer.elapsed()<ms);
}
static void mouse(QWidget* view,QEvent::Type type,QPoint p,Qt::MouseButton button,Qt::MouseButtons buttons={}) {
    QMouseEvent event(type,QPointF(p),QPointF(view->mapToGlobal(p)),button,buttons,Qt::NoModifier);
    QApplication::sendEvent(view,&event);
}
static void click(QWidget* view,QPoint p) {
    mouse(view,QEvent::MouseButtonPress,p,Qt::LeftButton,Qt::LeftButton);
    mouse(view,QEvent::MouseButtonRelease,p,Qt::LeftButton);pump();
}
static QCheckBox* box(CloudSelectionDialog& dialog,const QString& id) {
    for(auto* check:dialog.findChildren<QCheckBox*>()) if(check->text()==id) return check;
    return nullptr;
}
static QPushButton* pickButton(CloudSelectionDialog& dialog) {
    return dialog.findChild<QPushButton*>("viewportCloudSelectButton");
}
static void finish(CloudSelectionDialog& dialog,const QString& text) {
    for(auto* button:dialog.findChildren<QPushButton*>()) if(button->text()==text) {button->click();return;}
    dialog.reject();
}
static QPoint screen(QWidget* view,vtkRenderer* renderer,double x) {
    renderer->SetWorldPoint(x,0,0,1);renderer->WorldToDisplay();
    auto p=renderer->GetDisplayPoint();auto size=renderer->GetRenderWindow()->GetSize();
    return {qRound(p[0]*view->width()/size[0]),qRound((size[1]-1-p[1])*view->height()/size[1])};
}

int main(int argc,char** argv) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);
    QTimer::singleShot(0,&app,[&]() {
    CloudForgeAnalyzer main;main.resize(1400,950);main.show();pump(80);
    main.ClearAllPointCloud();
    const std::string a="前件",b="后件";
    for(auto entry:{std::pair<std::string,int>{a,-25},{b,25}}) {
        auto cloud=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for(int x=-4;x<=4;++x) for(int y=-4;y<=4;++y) cloud->emplace_back(entry.second+x,y,0);
        main.AddPointCloud(entry.first,cloud,entry.first==a?ColorManager(235,90,80):ColorManager(80,195,235));
    }
    auto* view=main.ui->winOfAnalyzer;
    auto* renderer=view->renderWindow()->GetRenderers()->GetFirstRenderer();
    auto* camera=renderer->GetActiveCamera();
    const auto reset=[&]() {
        camera->SetPosition(0,0,120);camera->SetFocalPoint(0,0,0);camera->SetViewUp(0,1,0);
        camera->ParallelProjectionOn();camera->SetParallelScale(38);
        renderer->ResetCameraClippingRange();view->renderWindow()->Render();pump();
    };
    reset();
    const auto cursor=view->cursor();const bool undoEnabled=main.ui->action_ed_undo->isEnabled();
    auto* selector=main.m_cloudSelector.get();
    const auto props=renderer->GetViewProps()->GetNumberOfItems();
    ChoseCloudDialog choose(main.CloudMap,main.ColorMap,"选择点云",&main);
    QTimer::singleShot(50,&choose,[&]() {
        choose.move(main.mapToGlobal(QPoint(100,65)));
        auto* button=pickButton(choose);
        check(button && button->isEnabled(),"selection subclass inherits shared corner button");
        check(!QApplication::activeModalWidget() && !main.ui->action_ed_undo->isEnabled(),
              "viewport is available while scene-edit actions are blocked");
        choose.showMinimized();pump();
        check(choose.selectionLoop_ && choose.selectionLoop_->isRunning(),"temporary window hiding does not finish the selection");
        choose.showNormal();pump();
        click(view,screen(view,renderer,-25));
        check(choose.getSelectedList().empty(),"view selection stays inactive until the button is enabled");
        button->click();click(view,screen(view,renderer,-25));
        check(box(choose,QString::fromStdString(a))->isChecked() && choose.getSelectedList().size()==1,
              "clicking cloud checks its matching row including Chinese ids");
        check(renderer->GetViewProps()->GetNumberOfItems()==props+1,"selected cloud has a visible non-pickable outline");
        click(view,screen(view,renderer,-25));
        check(choose.getSelectedList().empty(),"clicking the same cloud again unchecks the row");
        click(view,{2,view->height()-2});
        check(choose.getSelectedList().empty(),"empty-space click changes no row");
        box(choose,QString::fromStdString(a))->click();
        check(renderer->GetViewProps()->GetNumberOfItems()==props+1,"list checkbox updates viewport outline");
        click(view,screen(view,renderer,25));
        check(choose.getSelectedList().size()==2,"view picking supports multiple clouds");
        choose.clearAllCheckBoxes();
        check(choose.getSelectedList().empty() && renderer->GetViewProps()->GetNumberOfItems()==props,
              "clear-all also removes every outline");
        const auto p=screen(view,renderer,-25);
        double before[3];camera->GetPosition(before);
        mouse(view,QEvent::MouseButtonPress,p,Qt::LeftButton,Qt::LeftButton);
        mouse(view,QEvent::MouseMove,p+QPoint(55,30),Qt::NoButton,Qt::LeftButton);
        mouse(view,QEvent::MouseButtonRelease,p+QPoint(55,30),Qt::LeftButton);pump();
        const auto after=camera->GetPosition();
        check(choose.getSelectedList().empty() && (before[0]!=after[0]||before[1]!=after[1]),
              "camera drag rotates without selecting a cloud");
        reset();
        QKeyEvent escape(QEvent::KeyPress,Qt::Key_Escape,Qt::NoModifier);
        QApplication::sendEvent(view,&escape);
        check(choose.isVisible() && !button->isChecked(),"Escape leaves picking mode without discarding the dialog");
        button->click();click(view,screen(view,renderer,25));
        const auto actor=main.viewer->getCloudActorMap()->at(a).actor;
        actor->VisibilityOff();click(view,screen(view,renderer,-25));actor->VisibilityOn();
        check(choose.getSelectedList()==std::vector<std::string>{b},"hidden clouds cannot be selected");
        choose.grab().save("build/cloud-selection-dialog.png");
        main.grab().save("build/cloud-selection-viewport.png");
        finish(choose,QStringLiteral("确定"));
    });
    check(choose.exec()==QDialog::Accepted && choose.getSelectedList()==std::vector<std::string>{b},
          "confirmation returns viewport selection to the existing operation");
    check(renderer->GetViewProps()->GetNumberOfItems()==props && view->cursor().shape()==cursor.shape()
          && main.ui->action_ed_undo->isEnabled()==undoEnabled,"closing restores cursor, outlines and action states");

    QTimer::singleShot(50,&main,[&]() {
        auto dialogs=main.findChildren<CloudSelectionDialog*>();
        for(auto* dialog:dialogs) if(dialog->isVisible()) {
            pickButton(*dialog)->click();click(view,screen(view,renderer,-25));
            finish(*dialog,QStringLiteral("删除选中"));return;
        }
    });
    main.Slot_ed_dork_Triggered();
    check(main.CloudMap.size()==1 && main.CloudMap.count(b),"manager deletes exactly the cloud selected in the viewport");
    check(choose.getSelectedList()==std::vector<std::string>{b},"inactive earlier dialogs receive no new selection changes");

    QTimer::singleShot(50,&main,[&]() {
        for(auto* dialog:main.findChildren<CloudSelectionDialog*>()) if(dialog->isVisible()) {
            check(pickButton(*dialog)->isEnabled(),"save subclass inherits the same viewport selection button");
            pickButton(*dialog)->click();click(view,screen(view,renderer,25));
            finish(*dialog,QStringLiteral("另存为"));return;
        }
    });
    SaveCloudDialog save(main.CloudMap,main.ColorMap,&main);
    check(save.getSelectedList().empty(),"save selection toggles the default single-cloud check from the viewport");

    QTimer::singleShot(50,&main,[&]() {main.close();});
    ChoseCloudDialog closing(main.CloudMap,main.ColorMap,"关闭验证",&main);
    const auto closingResult=closing.exec();
    check(closingResult==QDialog::Rejected && !main.isVisible(),"closing the owner rejects the open selector and leaves no nested loop");
    std::cout<<"[SUMMARY] failures="<<failures<<std::endl;
    app.exit(failures?1:0);
    });
    return app.exec();
}
