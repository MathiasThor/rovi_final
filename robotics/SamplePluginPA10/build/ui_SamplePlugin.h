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
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
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
    QPushButton *_btn0;
    QPushButton *_startStopMovement;
    QPushButton *_followMarker;
    QPushButton *_resetSim;
    QFormLayout *formLayout_2;
    QComboBox *_comboBox;
    QDoubleSpinBox *_DT;
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
        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));

        verticalLayout->addWidget(_btn0);

        _startStopMovement = new QPushButton(dockWidgetContents);
        _startStopMovement->setObjectName(QString::fromUtf8("_startStopMovement"));

        verticalLayout->addWidget(_startStopMovement);

        _followMarker = new QPushButton(dockWidgetContents);
        _followMarker->setObjectName(QString::fromUtf8("_followMarker"));

        verticalLayout->addWidget(_followMarker);

        _resetSim = new QPushButton(dockWidgetContents);
        _resetSim->setObjectName(QString::fromUtf8("_resetSim"));

        verticalLayout->addWidget(_resetSim);

        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        formLayout_2->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        _comboBox = new QComboBox(dockWidgetContents);
        _comboBox->setObjectName(QString::fromUtf8("_comboBox"));
        _comboBox->setMinimumSize(QSize(0, 26));
        QFont font;
        font.setFamily(QString::fromUtf8("Waree"));
        font.setPointSize(11);
        font.setBold(false);
        font.setItalic(true);
        font.setWeight(50);
        _comboBox->setFont(font);
        _comboBox->setMaxVisibleItems(3);

        formLayout_2->setWidget(1, QFormLayout::LabelRole, _comboBox);

        _DT = new QDoubleSpinBox(dockWidgetContents);
        _DT->setObjectName(QString::fromUtf8("_DT"));
        _DT->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        _DT->setAccelerated(true);
        _DT->setDecimals(5);
        _DT->setMinimum(0);
        _DT->setMaximum(1);
        _DT->setSingleStep(0.001);
        _DT->setValue(1);

        formLayout_2->setWidget(1, QFormLayout::FieldRole, _DT);


        verticalLayout->addLayout(formLayout_2);

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
        _btn0->setText(QApplication::translate("SamplePlugin", "Load textures", 0, QApplication::UnicodeUTF8));
        _startStopMovement->setText(QApplication::translate("SamplePlugin", "Start / Stop movement", 0, QApplication::UnicodeUTF8));
        _followMarker->setText(QApplication::translate("SamplePlugin", "Follow marker", 0, QApplication::UnicodeUTF8));
        _resetSim->setText(QApplication::translate("SamplePlugin", "Reset", 0, QApplication::UnicodeUTF8));
        _comboBox->clear();
        _comboBox->insertItems(0, QStringList()
         << QApplication::translate("SamplePlugin", "MarkerMotionSlow.txt", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "MarkerMotionMedium.txt", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SamplePlugin", "MarkerMotionFast.txt", 0, QApplication::UnicodeUTF8)
        );
        _label->setText(QApplication::translate("SamplePlugin", "Label", 0, QApplication::UnicodeUTF8));
        _testRun->setText(QApplication::translate("SamplePlugin", "Test run", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
