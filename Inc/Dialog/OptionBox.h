#pragma once

#include <QDialog>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QLabel>
#include <QVector>
#include <QString>

/**
 * @brief 通用选项选择对话框
 *
 * 使用方法：
 * OptionBox box("选项1", "选项2", "选项3"); // 创建三个选项
 * box.setTitle("请选择");
 * box.setMessage("请从以下选项中选择一个：");
 *
 * if (box.exec() == QDialog::Accepted) {
 *     int selectedIndex = box.getSelectedIndex();
 *     QString selectedText = box.getSelectedText();
 *     // 处理选择
 * }
 */
class OptionBox : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数，创建包含指定选项的对话框
     * @param parent 父窗口
     * @param options 选项列表
     */
    explicit OptionBox(QWidget* parent = nullptr);
    OptionBox(const QStringList& options, QWidget* parent = nullptr);
    OptionBox(std::initializer_list<QString> options, QWidget* parent = nullptr);

    // 方便的函数，支持可变参数
    template<typename... Args>
    static OptionBox* create(Args... args) {
        return new OptionBox({ args... });
    }

    /**
     * @brief 执行对话框
     * @return 用户点击了某个选项返回 Accepted，取消或关闭返回 Rejected
     */
    int exec() override;

    // 设置对话框属性
    void setTitle(const QString& title);
    void setMessage(const QString& message);
    void setIcon(QMessageBox::Icon icon);
    void setDefaultOption(int index);

    // 获取选择结果
    int getSelectedIndex() const;
    QString getSelectedText() const;
    bool wasCancelled() const;

private slots:
    void onOptionClicked();
    void onCancelClicked();

private:
    void initUI();
    void addOption(const QString& text);

private:
    QVector<QPushButton*> m_optionButtons;
    QDialogButtonBox* m_buttonBox;
    QLabel* m_messageLabel;
    QVBoxLayout* m_mainLayout;

    QString m_title;
    QString m_message;
    QMessageBox::Icon m_icon;

    int m_selectedIndex;
    QString m_selectedText;
    bool m_cancelled;

    QStringList m_options;
};