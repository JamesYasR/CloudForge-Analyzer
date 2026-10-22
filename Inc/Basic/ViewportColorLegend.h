#pragma once
#include <QWidget>
#include <QGradient>
#include <QString>
#include <vector>

class QComboBox;
class ViewportColorLegend : public QWidget
{
public:
    struct Entry {
        QString id,title;
        double minimum=0,maximum=0;
        QGradientStops colors;
    };
    explicit ViewportColorLegend(QWidget* view);
    void setEntry(Entry entry);
    void removeEntry(const QString& id);
    void clear();
    static QGradientStops sequential();
    static QGradientStops signedHeight();
    static QGradientStops signedSurface();
protected:
    void paintEvent(QPaintEvent*) override;
    bool eventFilter(QObject*,QEvent*) override;
private:
    void updateLayout();
    QComboBox* selector_;
    std::vector<Entry> entries_;
};
