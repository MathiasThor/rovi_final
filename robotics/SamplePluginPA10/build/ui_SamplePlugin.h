/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QFrame *line_2;
    QLabel *label;
    QHBoxLayout *horizontalLayout_2;
    QComboBox *_backTex;
    QComboBox *_markTex;
    QPushButton *_btn0;
    QFrame *line;
    QLabel *label_2;
    QPushButton *_startStopMovement;
    QPushButton *_followMarker;
    QPushButton *_resetSim;
    QFrame *line_3;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_3;
    QComboBox *_comboBox;
    QDoubleSpinBox *_DT;
    QFrame *line_4;
    QLabel *_label;
    QSpacerItem *verticalSpacer;
    QPushButton *_testRun;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(428, 615);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        line_2 = new QFrame(dockWidgetContents);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_2);

        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _backTex = new QComboBox(dockWidgetContents);
        _backTex->setObjectName(QString::fromUtf8("_backTex"));

        horizontalLayout_2->addWidget(_backTex);

        _markTex = new QComboBox(dockWidgetContents);
        _markTex->setObjectName(QString::fromUtf8("_markTex"));

        horizontalLayout_2->addWidget(_markTex);


        verticalLayout->addLayout(horizontalLayout_2);

        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));

        verticalLayout->addWidget(_btn0);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        _startStopMovement = new QPushButton(dockWidgetContents);
        _startStopMovement->setObjectName(QString::fromUtf8("_startStopMovement"));

        verticalLayout->addWidget(_startStopMovement);

        _followMarker = new QPushButton(dockWidgetContents);
        _followMarker->setObjectName(QString::fromUtf8("_followMarker"));

        verticalLayout->addWidget(_followMarker);

        _resetSim = new QPushButton(dockWidgetContents);
        _resetSim->setObjectName(QString::fromUtf8("_resetSim"));

        verticalLayout->addWidget(_resetSim);

        line_3 = new QFrame(dockWidgetContents);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_3);

        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout->addWidget(label_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        _comboBox = new QComboBox(dockWidgetContents);
        _comboBox->setObjectName(QString::fromUtf8("_comboBox"));

        horizontalLayout_3->addWidget(_comboBox);

        _DT = new QDoubleSpinBox(dockWidgetContents);
        _DT->setObjectName(QString::fromUtf8("_DT"));
        _DT->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        _DT->setAccelerated(true);
        _DT->setDecimals(5);
        _DT->setMinimum(0);
        _DT->setMaximum(1);
        _DT->setSingleStep(0.001);
        _DT->setValue(1);

        horizontalLayout_3->addWidget(_DT);


        verticalLayout->addLayout(horizontalLayout_3);

        line_4 = new QFrame(dockWidgetContents);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_4);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QString::fromUtf8("_label"));

        verticalLayout->addWidget(_label);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        _testRun = new QPushButton(dockWidgetContents);
        _testRun->setObjectName(QString::fromUtf8("_testRun"));

        verticalLayout->addWidget(_testRun);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SamplePlugin", "<html><head/><body><p align=\"center\"><span style=\" font-size:8pt; color:#9a9a9a;\">TEXTURE LOADER</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        _backTex->clear();
        _backTex->insertItems(0, QStringList()
         << QApplication::translate("SamplePlugin", "color1.ppm", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "color2.ppm", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "color3.ppm", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "lines1.ppm", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "texture1.ppm", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "texture2.ppm", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "texture3.ppm", 0, QApplication::UnicodeUTF8)
        );
        _markTex->clear();
        _markTex->insertItems(0, QStringList()
         << QApplication::translate("SamplePlugin", "Marker1.ppm", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "Marker3.ppm", 0, QApplication::UnicodeUTF8)
        );
        _btn0->setText(QApplication::translate("SamplePlugin", "Load textures", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SamplePlugin", "<html><head/><body><p align=\"center\"><span style=\" font-size:8pt; color:#9a9a9a;\">ROBOT CONTROL</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        _startStopMovement->setText(QApplication::translate("SamplePlugin", "Start / Stop movement", 0, QApplication::UnicodeUTF8));
        _followMarker->setText(QApplication::translate("SamplePlugin", "Follow one frame", 0, QApplication::UnicodeUTF8));
        _resetSim->setText(QApplication::translate("SamplePlugin", "Reset", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SamplePlugin", "<html><head/><body><p align=\"center\"><span style=\" font-size:8pt; color:#9a9a9a;\">MOTION &amp; DELTA T</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        _comboBox->clear();
        _comboBox->insertItems(0, QStringList()
         << QApplication::translate("SamplePlugin", "MarkerMotionSlow.txt", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "MarkerMotionMedium.txt", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "MarkerMotionFast.txt", 0, QApplication::UnicodeUTF8)
        );
        _label->setText(QApplication::translate("SamplePlugin", "<html><head/><body><p align=\"center\"/><p align=\"center\"/><p align=\"center\"><span style=\" font-style:italic; color:#8a0000;\">Start movement to display robot camera</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        _testRun->setText(QApplication::translate("SamplePlugin", "Test run", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
