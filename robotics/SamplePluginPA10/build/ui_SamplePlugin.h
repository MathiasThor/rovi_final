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
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
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
    QCheckBox *_checkBox;
    QSpinBox *_spinBox;
    QSlider *_slider;
    QLabel *_label;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(428, 475);
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

        _checkBox = new QCheckBox(dockWidgetContents);
        _checkBox->setObjectName(QString::fromUtf8("_checkBox"));

        verticalLayout->addWidget(_checkBox);

        _spinBox = new QSpinBox(dockWidgetContents);
        _spinBox->setObjectName(QString::fromUtf8("_spinBox"));

        verticalLayout->addWidget(_spinBox);

        _slider = new QSlider(dockWidgetContents);
        _slider->setObjectName(QString::fromUtf8("_slider"));
        _slider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(_slider);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QString::fromUtf8("_label"));

        verticalLayout->addWidget(_label);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "DockWidget", 0, QApplication::UnicodeUTF8));
        _btn0->setText(QApplication::translate("SamplePlugin", "PushButton0", 0, QApplication::UnicodeUTF8));
        _btn1->setText(QApplication::translate("SamplePlugin", "PushButton1", 0, QApplication::UnicodeUTF8));
        _checkBox->setText(QApplication::translate("SamplePlugin", "CheckBox", 0, QApplication::UnicodeUTF8));
        _label->setText(QApplication::translate("SamplePlugin", "Label", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
