/****************************************************************************
** Meta object code from reading C++ file 'motor_control_panel.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ui/motor_control_panel.h"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'motor_control_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__MotorControlPanel_t {
    const uint offsetsAndSize[46];
    char stringdata0[359];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__MotorControlPanel_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__MotorControlPanel_t qt_meta_stringdata_dac__MotorControlPanel = {
    {
QT_MOC_LITERAL(0, 22), // "dac::MotorControlPanel"
QT_MOC_LITERAL(23, 15), // "enableRequested"
QT_MOC_LITERAL(39, 0), // ""
QT_MOC_LITERAL(40, 8), // "uint32_t"
QT_MOC_LITERAL(49, 6), // "nodeId"
QT_MOC_LITERAL(56, 18), // "dac::OperationMode"
QT_MOC_LITERAL(75, 4), // "mode"
QT_MOC_LITERAL(80, 16), // "disableRequested"
QT_MOC_LITERAL(97, 22), // "emergencyStopRequested"
QT_MOC_LITERAL(120, 19), // "faultResetRequested"
QT_MOC_LITERAL(140, 17), // "velocityRequested"
QT_MOC_LITERAL(158, 7), // "int32_t"
QT_MOC_LITERAL(166, 3), // "rpm"
QT_MOC_LITERAL(170, 17), // "positionRequested"
QT_MOC_LITERAL(188, 7), // "degrees"
QT_MOC_LITERAL(196, 8), // "absolute"
QT_MOC_LITERAL(205, 24), // "profileVelocityRequested"
QT_MOC_LITERAL(230, 21), // "releaseBrakeRequested"
QT_MOC_LITERAL(252, 18), // "lockBrakeRequested"
QT_MOC_LITERAL(271, 18), // "enableAllRequested"
QT_MOC_LITERAL(290, 19), // "disableAllRequested"
QT_MOC_LITERAL(310, 25), // "emergencyStopAllRequested"
QT_MOC_LITERAL(336, 22) // "faultResetAllRequested"

    },
    "dac::MotorControlPanel\0enableRequested\0"
    "\0uint32_t\0nodeId\0dac::OperationMode\0"
    "mode\0disableRequested\0emergencyStopRequested\0"
    "faultResetRequested\0velocityRequested\0"
    "int32_t\0rpm\0positionRequested\0degrees\0"
    "absolute\0profileVelocityRequested\0"
    "releaseBrakeRequested\0lockBrakeRequested\0"
    "enableAllRequested\0disableAllRequested\0"
    "emergencyStopAllRequested\0"
    "faultResetAllRequested"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__MotorControlPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      13,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   92,    2, 0x06,    1 /* Public */,
       7,    1,   97,    2, 0x06,    4 /* Public */,
       8,    1,  100,    2, 0x06,    6 /* Public */,
       9,    1,  103,    2, 0x06,    8 /* Public */,
      10,    2,  106,    2, 0x06,   10 /* Public */,
      13,    3,  111,    2, 0x06,   13 /* Public */,
      16,    2,  118,    2, 0x06,   17 /* Public */,
      17,    1,  123,    2, 0x06,   20 /* Public */,
      18,    1,  126,    2, 0x06,   22 /* Public */,
      19,    1,  129,    2, 0x06,   24 /* Public */,
      20,    0,  132,    2, 0x06,   26 /* Public */,
      21,    0,  133,    2, 0x06,   27 /* Public */,
      22,    0,  134,    2, 0x06,   28 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 11,    4,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Double, QMetaType::Bool,    4,   14,   15,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,   12,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void dac::MotorControlPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MotorControlPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->enableRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<dac::OperationMode>>(_a[2]))); break;
        case 1: _t->disableRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1]))); break;
        case 2: _t->emergencyStopRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1]))); break;
        case 3: _t->faultResetRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1]))); break;
        case 4: _t->velocityRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int32_t>>(_a[2]))); break;
        case 5: _t->positionRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[3]))); break;
        case 6: _t->profileVelocityRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[2]))); break;
        case 7: _t->releaseBrakeRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1]))); break;
        case 8: _t->lockBrakeRequested((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1]))); break;
        case 9: _t->enableAllRequested((*reinterpret_cast< std::add_pointer_t<dac::OperationMode>>(_a[1]))); break;
        case 10: _t->disableAllRequested(); break;
        case 11: _t->emergencyStopAllRequested(); break;
        case 12: _t->faultResetAllRequested(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MotorControlPanel::*)(uint32_t , dac::OperationMode );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::enableRequested)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::disableRequested)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::emergencyStopRequested)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::faultResetRequested)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t , int32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::velocityRequested)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t , double , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::positionRequested)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t , uint32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::profileVelocityRequested)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::releaseBrakeRequested)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(uint32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::lockBrakeRequested)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)(dac::OperationMode );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::enableAllRequested)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::disableAllRequested)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::emergencyStopAllRequested)) {
                *result = 11;
                return;
            }
        }
        {
            using _t = void (MotorControlPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorControlPanel::faultResetAllRequested)) {
                *result = 12;
                return;
            }
        }
    }
}

const QMetaObject dac::MotorControlPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_dac__MotorControlPanel.offsetsAndSize,
    qt_meta_data_dac__MotorControlPanel,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__MotorControlPanel_t
, QtPrivate::TypeAndForceComplete<MotorControlPanel, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<dac::OperationMode, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<int32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::OperationMode, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>



>,
    nullptr
} };


const QMetaObject *dac::MotorControlPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::MotorControlPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__MotorControlPanel.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int dac::MotorControlPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void dac::MotorControlPanel::enableRequested(uint32_t _t1, dac::OperationMode _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::MotorControlPanel::disableRequested(uint32_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void dac::MotorControlPanel::emergencyStopRequested(uint32_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void dac::MotorControlPanel::faultResetRequested(uint32_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void dac::MotorControlPanel::velocityRequested(uint32_t _t1, int32_t _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void dac::MotorControlPanel::positionRequested(uint32_t _t1, double _t2, bool _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void dac::MotorControlPanel::profileVelocityRequested(uint32_t _t1, uint32_t _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void dac::MotorControlPanel::releaseBrakeRequested(uint32_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void dac::MotorControlPanel::lockBrakeRequested(uint32_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void dac::MotorControlPanel::enableAllRequested(dac::OperationMode _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void dac::MotorControlPanel::disableAllRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 10, nullptr);
}

// SIGNAL 11
void dac::MotorControlPanel::emergencyStopAllRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 11, nullptr);
}

// SIGNAL 12
void dac::MotorControlPanel::faultResetAllRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 12, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
