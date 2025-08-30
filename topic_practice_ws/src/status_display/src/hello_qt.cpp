#include <QApplication>
#include <QLabel>
#include <QString>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QLabel* label = new QLabel();
    QString message = QString::fromStdString("Hello, Qt!");
    
    label->setText(message);
    label->resize(300, 120);  // 宽=200像素，高=100像素
    label->show();
    app.exec();

    return 0;
}