/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created: Fri Jul 4 11:24:07 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralWidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QHBoxLayout *horizontalLayout;
    QSlider *horizontalSlider_R;
    QLCDNumber *redSliderValueChanged;
    QPushButton *pushButton_random;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_2;
    QSlider *horizontalSlider_G;
    QLCDNumber *greenSliderValueChanged;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_3;
    QSlider *horizontalSlider_B;
    QLCDNumber *blueSliderValueChanged;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_4;
    QSlider *horizontalSlider_p;
    QLCDNumber *pSliderValueChanged;
    QWidget *qvtkWidget;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QString::fromUtf8("PCLViewer"));
        PCLViewer->resize(599, 424);
        centralWidget = new QWidget(PCLViewer);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(40, 10, 191, 81));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(verticalLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSlider_R = new QSlider(verticalLayoutWidget);
        horizontalSlider_R->setObjectName(QString::fromUtf8("horizontalSlider_R"));
        horizontalSlider_R->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(horizontalSlider_R);

        redSliderValueChanged = new QLCDNumber(verticalLayoutWidget);
        redSliderValueChanged->setObjectName(QString::fromUtf8("redSliderValueChanged"));

        horizontalLayout->addWidget(redSliderValueChanged);


        verticalLayout->addLayout(horizontalLayout);

        pushButton_random = new QPushButton(centralWidget);
        pushButton_random->setObjectName(QString::fromUtf8("pushButton_random"));
        pushButton_random->setGeometry(QRect(350, 280, 141, 51));
        pushButton_random->setContextMenuPolicy(Qt::DefaultContextMenu);
        verticalLayoutWidget_2 = new QWidget(centralWidget);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(40, 90, 191, 81));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(verticalLayoutWidget_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalSlider_G = new QSlider(verticalLayoutWidget_2);
        horizontalSlider_G->setObjectName(QString::fromUtf8("horizontalSlider_G"));
        horizontalSlider_G->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(horizontalSlider_G);

        greenSliderValueChanged = new QLCDNumber(verticalLayoutWidget_2);
        greenSliderValueChanged->setObjectName(QString::fromUtf8("greenSliderValueChanged"));

        horizontalLayout_2->addWidget(greenSliderValueChanged);


        verticalLayout_2->addLayout(horizontalLayout_2);

        verticalLayoutWidget_3 = new QWidget(centralWidget);
        verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(40, 170, 191, 81));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(verticalLayoutWidget_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_3->addWidget(label_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalSlider_B = new QSlider(verticalLayoutWidget_3);
        horizontalSlider_B->setObjectName(QString::fromUtf8("horizontalSlider_B"));
        horizontalSlider_B->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(horizontalSlider_B);

        blueSliderValueChanged = new QLCDNumber(verticalLayoutWidget_3);
        blueSliderValueChanged->setObjectName(QString::fromUtf8("blueSliderValueChanged"));

        horizontalLayout_3->addWidget(blueSliderValueChanged);


        verticalLayout_3->addLayout(horizontalLayout_3);

        verticalLayoutWidget_4 = new QWidget(centralWidget);
        verticalLayoutWidget_4->setObjectName(QString::fromUtf8("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(40, 250, 191, 81));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(verticalLayoutWidget_4);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_4->addWidget(label_4);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalSlider_p = new QSlider(verticalLayoutWidget_4);
        horizontalSlider_p->setObjectName(QString::fromUtf8("horizontalSlider_p"));
        horizontalSlider_p->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(horizontalSlider_p);

        pSliderValueChanged = new QLCDNumber(verticalLayoutWidget_4);
        pSliderValueChanged->setObjectName(QString::fromUtf8("pSliderValueChanged"));

        horizontalLayout_4->addWidget(pSliderValueChanged);


        verticalLayout_4->addLayout(horizontalLayout_4);

        qvtkWidget = new QWidget(centralWidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(270, 10, 291, 261));
        PCLViewer->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PCLViewer);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 599, 25));
        PCLViewer->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PCLViewer);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        PCLViewer->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PCLViewer);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        PCLViewer->setStatusBar(statusBar);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("PCLViewer", "Red Component", 0, QApplication::UnicodeUTF8));
        pushButton_random->setText(QApplication::translate("PCLViewer", "Random Colors", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("PCLViewer", "Green Component", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("PCLViewer", "Blue Component", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("PCLViewer", "Point Size", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
