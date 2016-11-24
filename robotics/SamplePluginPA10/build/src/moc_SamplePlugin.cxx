/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SamplePlugin.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SamplePlugin[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x08,
      27,   13,   13,   13, 0x08,
      41,   35,   13,   13, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SamplePlugin[] = {
    "SamplePlugin\0\0btnPressed()\0timer()\0"
    "state\0stateChangedListener(rw::kinematics::State)\0"
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SamplePlugin *_t = static_cast<SamplePlugin *>(_o);
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->timer(); break;
        case 2: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SamplePlugin::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SamplePlugin::staticMetaObject = {
    { &rws::RobWorkStudioPlugin::staticMetaObject, qt_meta_stringdata_SamplePlugin,
      qt_meta_data_SamplePlugin, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SamplePlugin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin))
        return static_cast<void*>(const_cast< SamplePlugin*>(this));
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(const_cast< SamplePlugin*>(this));
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
