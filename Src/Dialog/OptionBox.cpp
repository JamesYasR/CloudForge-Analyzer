#include "Dialog/OptionBox.h"
#include <QApplication>
#include <QStyle>

OptionBox::OptionBox(QWidget* parent)
    : QDialog(parent)
    , m_selectedIndex(-1)
    , m_cancelled(true)
    , m_icon(QMessageBox::NoIcon)
{
    initUI();
}

OptionBox::OptionBox(const QStringList& options, QWidget* parent)
    : QDialog(parent)
    , m_options(options)
    , m_selectedIndex(-1)
    , m_cancelled(true)
    , m_icon(QMessageBox::Question)
{
    initUI();
    for (const QString& option : options) {
        addOption(option);
    }
}

OptionBox::OptionBox(std::initializer_list<QString> options, QWidget* parent)
    : QDialog(parent)
    , m_options(options)
    , m_selectedIndex(-1)
    , m_cancelled(true)
    , m_icon(QMessageBox::Question)
{
    initUI();
    for (const QString& option : options) {
        addOption(option);
    }
}

void OptionBox::initUI()
{
    setWindowTitle("请选择");
    m_message = "请选择一个选项：";

    // 设置对话框属性
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    setWindowModality(Qt::ApplicationModal);

    // 创建主布局
    m_mainLayout = new QVBoxLayout(this);

    // 创建消息标签
    m_messageLabel = new QLabel(m_message, this);
    m_messageLabel->setWordWrap(true);
    m_messageLabel->setAlignment(Qt::AlignCenter);
    m_mainLayout->addWidget(m_messageLabel);

    // 创建按钮容器
    QWidget* buttonContainer = new QWidget(this);
    QVBoxLayout* buttonLayout = new QVBoxLayout(buttonContainer);
    buttonLayout->setSpacing(8);
    buttonLayout->setContentsMargins(20, 10, 20, 10);

    // 设置按钮容器布局
    buttonContainer->setLayout(buttonLayout);
    m_mainLayout->addWidget(buttonContainer);

    // 创建取消按钮区域
    m_buttonBox = new QDialogButtonBox(QDialogButtonBox::Cancel, this);
    m_buttonBox->button(QDialogButtonBox::Cancel)->setText("取消");

    // 连接取消按钮信号
    connect(m_buttonBox, &QDialogButtonBox::rejected, this, &OptionBox::onCancelClicked);

    m_mainLayout->addWidget(m_buttonBox);

    // 设置主布局
    setLayout(m_mainLayout);

    // 设置默认大小
    resize(300, 200);
}

void OptionBox::addOption(const QString& text)
{
    QPushButton* button = new QPushButton(text, this);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    button->setMinimumHeight(40);

    // 应用样式使其看起来像对话框按钮
    button->setStyleSheet(
        "QPushButton {"
        "    background-color: #f0f0f0;"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 8px;"
        "    font-size: 14px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #e0e0e0;"
        "    border-color: #aaaaaa;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #d0d0d0;"
        "}"
    );

    // 找到按钮容器
    QWidget* buttonContainer = qobject_cast<QWidget*>(m_mainLayout->itemAt(1)->widget());
    if (buttonContainer) {
        QVBoxLayout* buttonLayout = qobject_cast<QVBoxLayout*>(buttonContainer->layout());
        if (buttonLayout) {
            buttonLayout->insertWidget(buttonLayout->count() - 1, button);
        }
    }

    // 连接按钮信号
    connect(button, &QPushButton::clicked, this, &OptionBox::onOptionClicked);

    m_optionButtons.append(button);
}

int OptionBox::exec()
{
    // 重置选择状态
    m_selectedIndex = -1;
    m_selectedText.clear();
    m_cancelled = true;

    return QDialog::exec();
}

void OptionBox::onOptionClicked()
{
    QPushButton* button = qobject_cast<QPushButton*>(sender());
    if (!button) {
        return;
    }

    // 找到被点击按钮的索引
    m_selectedIndex = m_optionButtons.indexOf(button);
    if (m_selectedIndex >= 0) {
        m_selectedText = button->text();
        m_cancelled = false;
    }

    accept();
}

void OptionBox::onCancelClicked()
{
    m_cancelled = true;
    m_selectedIndex = -1;
    m_selectedText.clear();
    reject();
}

void OptionBox::setTitle(const QString& title)
{
    m_title = title;
    setWindowTitle(title);
}

void OptionBox::setMessage(const QString& message)
{
    m_message = message;
    m_messageLabel->setText(message);
}

void OptionBox::setIcon(QMessageBox::Icon icon)
{
    m_icon = icon;

    // 根据图标设置窗口图标
    switch (icon) {
    case QMessageBox::Question:
        setWindowIcon(QApplication::style()->standardIcon(QStyle::SP_MessageBoxQuestion));
        break;
    case QMessageBox::Information:
        setWindowIcon(QApplication::style()->standardIcon(QStyle::SP_MessageBoxInformation));
        break;
    case QMessageBox::Warning:
        setWindowIcon(QApplication::style()->standardIcon(QStyle::SP_MessageBoxWarning));
        break;
    case QMessageBox::Critical:
        setWindowIcon(QApplication::style()->standardIcon(QStyle::SP_MessageBoxCritical));
        break;
    default:
        setWindowIcon(QIcon());
        break;
    }
}

void OptionBox::setDefaultOption(int index)
{
    if (index >= 0 && index < m_optionButtons.size()) {
        m_optionButtons[index]->setDefault(true);
        m_optionButtons[index]->setFocus();
    }
}

int OptionBox::getSelectedIndex() const
{
    return m_selectedIndex;
}

QString OptionBox::getSelectedText() const
{
    return m_selectedText;
}

bool OptionBox::wasCancelled() const
{
    return m_cancelled;
}