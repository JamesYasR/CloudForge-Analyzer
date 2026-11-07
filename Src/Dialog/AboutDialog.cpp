#include "Dialog/AboutDialog.h"

#include <QApplication>
#include <QClipboard>
#include <QDialogButtonBox>
#include <QFile>
#include <QHBoxLayout>
#include <QIcon>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace {
QString projectVersion()
{
    // Embed the existing manifest so installed builds use the same version source.
    QFile manifest(QStringLiteral(":/CloudForgeAnalyzer/project-metadata.json"));
    if (manifest.open(QIODevice::ReadOnly)) {
        const auto version = QJsonDocument::fromJson(manifest.readAll())
                                 .object().value(QStringLiteral("version-string")).toString();
        if (!version.isEmpty()) return version;
    }
    return QStringLiteral("未知版本");
}
}

AboutDialog::AboutDialog(QWidget* parent) : QDialog(parent)
{
    setObjectName(QStringLiteral("aboutDialog"));
    setWindowTitle(QStringLiteral("关于 CloudForge Analyzer"));
    const QIcon icon(QStringLiteral(":/CloudForgeAnalyzer/Icons/icon_MainWindow.png"));
    setWindowIcon(icon);
    setMinimumWidth(460);
    resize(490, 340);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(24, 24, 24, 20);
    layout->setSpacing(18);

    const QString version = projectVersion();
    auto* header = new QHBoxLayout;
    header->setSpacing(16);
    auto* logo = new QLabel(this);
    logo->setPixmap(icon.pixmap(52, 52));
    header->addWidget(logo);
    auto* heading = new QVBoxLayout;
    heading->setSpacing(4);
    auto* title = new QLabel(QStringLiteral("CloudForge Analyzer"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(18);
    titleFont.setWeight(QFont::DemiBold);
    title->setFont(titleFont);
    heading->addWidget(title);
    heading->addWidget(new QLabel(QStringLiteral("版本 %1").arg(version), this));
    header->addLayout(heading, 1);
    layout->addLayout(header);

    auto* description = new QLabel(
        QStringLiteral("面向工程点云的处理、几何拟合与测量分析工具。"), this);
    description->setWordWrap(true);
    layout->addWidget(description);

    auto* details = new QVBoxLayout;
    details->setSpacing(8);
    details->addWidget(new QLabel(QStringLiteral("开发与维护：JamesYasR"), this));
    auto* credits = new QLabel(
        QStringLiteral("基于 Qt、PCL 与 VTK 构建，感谢相关开源项目。"), this);
    credits->setWordWrap(true);
    details->addWidget(credits);
    auto* videoNote = new QLabel(QStringLiteral("相关操作请看视频案例。"), this);
    videoNote->setWordWrap(true);
    details->addWidget(videoNote);
    auto* homepage = new QLabel(QStringLiteral(
        "<a href=\"https://github.com/JamesYasR/CloudForge-Analyzer\">项目主页 ↗</a>"), this);
    homepage->setTextInteractionFlags(Qt::TextBrowserInteraction);
    homepage->setOpenExternalLinks(true);
    //details->addWidget(homepage);
    layout->addLayout(details);
    layout->addStretch();

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    auto* copy = buttons->addButton(QStringLiteral("复制版本信息"), QDialogButtonBox::ActionRole);
    copy->setObjectName(QStringLiteral("copyVersionButton"));
    auto* close = buttons->button(QDialogButtonBox::Close);
    close->setText(QStringLiteral("关闭"));
    close->setDefault(true);
    close->setFocus();
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(copy, &QPushButton::clicked, this, [this, copy, version] {
        QApplication::clipboard()->setText(QStringLiteral(
            "CloudForge Analyzer %1\n开发与维护：JamesYasR\n"
            "https://github.com/JamesYasR/CloudForge-Analyzer").arg(version));
        copy->setText(QStringLiteral("已复制"));
        QTimer::singleShot(1800, this, [copy] {
            copy->setText(QStringLiteral("复制版本信息"));
        });
    });
    layout->addWidget(buttons);
}
