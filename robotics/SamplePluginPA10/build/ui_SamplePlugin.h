/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *_btn0;
    QPushButton *_btn1;
    QPushButton *_startStopMovement;
    QSpinBox *_spinBox;
    QLabel *_label;
    QSpacerItem *verticalSpacer;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(428, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));

        verticalLayout->addWidget(_btn0);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QString::fromUtf8("_btn1"));

        verticalLayout->addWidget(_btn1);

        _startStopMovement = new QPushButton(dockWidgetContents);
        _startStopMovement->setObjectName(QString::fromUtf8("_startStopMovement"));

        verticalLayout->addWidget(_startStopMovement);

        _spinBox = new QSpinBox(dockWidgetContents);
        _spinBox->setObjectName(QString::fromUtf8("_spinBox"));
        QFont font;
        font.setPointSize(10);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        _spinBox->setFont(font);
        _spinBox->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
        _spinBox->setMinimum(1);
        _spinBox->setMaximum(4);

        verticalLayout->addWidget(_spinBox);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QString::fromUtf8("_label"));

        verticalLayout->addWidget(_label, 0, Qt::AlignHCenter);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", 0, QApplication::UnicodeUTF8));
        _btn0->setText(QApplication::translate("SamplePlugin", "Load textures", 0, QApplication::UnicodeUTF8));
        _btn1->setText(QApplication::translate("SamplePlugin", "Timer / Update Video", 0, QApplication::UnicodeUTF8));
        _startStopMovement->setText(QApplication::translate("SamplePlugin", "Start / Stop movement", 0, QApplication::UnicodeUTF8));
        _label->setText(QApplication::translate("SamplePlugin", "Label", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
