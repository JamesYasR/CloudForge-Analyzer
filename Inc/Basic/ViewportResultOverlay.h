#pragma once

#include <QWidget>
#include <QColor>
#include <QPointer>
#include <vector>

class QPainter;

class ViewportResultOverlay : public QWidget
{
    Q_OBJECT
public:
    struct Row {
        QString label;
        QString value;
        QColor color;
    };
    struct Section {
        QString title;
        std::vector<Row> rows;
        int columns = 1;
    };

    explicit ViewportResultOverlay(QWidget* view);
    void setSection(const QString& id, Section section);
    void removeSection(const QString& id);
    void clear();
    void avoidWidget(QWidget* widget);
    QRect occupiedRect() const;
    static QColor backgroundColor();
    static QColor borderColor();
    static QColor textColor();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void paintEvent(QPaintEvent*) override;

private:
    struct Entry {
        QString id;
        Section section;
    };
    int layoutContents(QPainter* painter) const;
    void updateLayout();
    std::vector<Entry> sections_;
    std::vector<QPointer<QWidget>> obstacles_;
};
