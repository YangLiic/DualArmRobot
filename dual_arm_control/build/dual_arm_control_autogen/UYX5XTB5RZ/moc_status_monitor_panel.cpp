/****************************************************************************
** Meta object code from reading C++ file 'status_monitor_panel.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ui/status_monitor_panel.h"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'status_monitor_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__StatusMonitorPanel_t {
    const uint offsetsAndSize[22];
    char stringdata0[139];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__StatusMonitorPanel_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__StatusMonitorPanel_t qt_meta_stringdata_dac__StatusMonitorPanel = {
    {
QT_MOC_LITERAL(0, 23), // "dac::StatusMonitorPanel"
QT_MOC_LITERAL(24, 16), // "updateMotorState"
QT_MOC_LITERAL(41, 0), // ""
QT_MOC_LITERAL(42, 8), // "uint32_t"
QT_MOC_LITERAL(51, 6), // "nodeId"
QT_MOC_LITERAL(58, 15), // "dac::MotorState"
QT_MOC_LITERAL(74, 5), // "state"
QT_MOC_LITERAL(80, 15), // "onTorqueUpdated"
QT_MOC_LITERAL(96, 7), // "int16_t"
QT_MOC_LITERAL(104, 14), // "torquePermille"
QT_MOC_LITERAL(119, 19) // "onCollisionDetected"

    },
    "dac::StatusMonitorPanel\0updateMotorState\0"
    "\0uint32_t\0nodeId\0dac::MotorState\0state\0"
    "onTorqueUpdated\0int16_t\0torquePermille\0"
    "onCollisionDetected"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__StatusMonitorPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   32,    2, 0x0a,    1 /* Public */,
       7,    2,   37,    2, 0x0a,    4 /* Public */,
      10,    2,   42,    2, 0x0a,    7 /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 8,    4,    9,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 8,    4,    9,

       0        // eod
};

void dac::StatusMonitorPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<StatusMonitorPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateMotorState((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<dac::MotorState>>(_a[2]))); break;
        case 1: _t->onTorqueUpdated((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int16_t>>(_a[2]))); break;
        case 2: _t->onCollisionDetected((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int16_t>>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObject dac::StatusMonitorPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_dac__StatusMonitorPanel.offsetsAndSize,
    qt_meta_data_dac__StatusMonitorPanel,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__StatusMonitorPanel_t
, QtPrivate::TypeAndForceComplete<StatusMonitorPanel, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::MotorState &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<int16_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<int16_t, std::false_type>


>,
    nullptr
} };


const QMetaObject *dac::StatusMonitorPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::StatusMonitorPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__StatusMonitorPanel.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int dac::StatusMonitorPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
