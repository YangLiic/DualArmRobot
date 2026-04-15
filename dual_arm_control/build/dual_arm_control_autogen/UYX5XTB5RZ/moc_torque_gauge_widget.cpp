/****************************************************************************
** Meta object code from reading C++ file 'torque_gauge_widget.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ui/torque_gauge_widget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'torque_gauge_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__TorqueGaugeWidget_t {
    const uint offsetsAndSize[26];
    char stringdata0[154];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__TorqueGaugeWidget_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__TorqueGaugeWidget_t qt_meta_stringdata_dac__TorqueGaugeWidget = {
    {
QT_MOC_LITERAL(0, 22), // "dac::TorqueGaugeWidget"
QT_MOC_LITERAL(23, 14), // "setTorqueValue"
QT_MOC_LITERAL(38, 0), // ""
QT_MOC_LITERAL(39, 8), // "permille"
QT_MOC_LITERAL(48, 9), // "setOnline"
QT_MOC_LITERAL(58, 6), // "online"
QT_MOC_LITERAL(65, 10), // "setEnabled"
QT_MOC_LITERAL(76, 7), // "enabled"
QT_MOC_LITERAL(84, 21), // "setCollisionTriggered"
QT_MOC_LITERAL(106, 9), // "triggered"
QT_MOC_LITERAL(116, 22), // "setCollisionProtection"
QT_MOC_LITERAL(139, 2), // "on"
QT_MOC_LITERAL(142, 11) // "torqueValue"

    },
    "dac::TorqueGaugeWidget\0setTorqueValue\0"
    "\0permille\0setOnline\0online\0setEnabled\0"
    "enabled\0setCollisionTriggered\0triggered\0"
    "setCollisionProtection\0on\0torqueValue"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__TorqueGaugeWidget[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       1,   59, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   44,    2, 0x0a,    2 /* Public */,
       4,    1,   47,    2, 0x0a,    4 /* Public */,
       6,    1,   50,    2, 0x0a,    6 /* Public */,
       8,    1,   53,    2, 0x0a,    8 /* Public */,
      10,    1,   56,    2, 0x0a,   10 /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void, QMetaType::Bool,   11,

 // properties: name, type, flags
      12, QMetaType::Int, 0x00015103, uint(-1), 0,

       0        // eod
};

void dac::TorqueGaugeWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TorqueGaugeWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->setTorqueValue((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 1: _t->setOnline((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 2: _t->setEnabled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 3: _t->setCollisionTriggered((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 4: _t->setCollisionProtection((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        default: ;
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<TorqueGaugeWidget *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< int*>(_v) = _t->torqueValue(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<TorqueGaugeWidget *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: _t->setTorqueValue(*reinterpret_cast< int*>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    } else if (_c == QMetaObject::BindableProperty) {
    }
#endif // QT_NO_PROPERTIES
}

const QMetaObject dac::TorqueGaugeWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_dac__TorqueGaugeWidget.offsetsAndSize,
    qt_meta_data_dac__TorqueGaugeWidget,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__TorqueGaugeWidget_t
, QtPrivate::TypeAndForceComplete<int, std::true_type>, QtPrivate::TypeAndForceComplete<TorqueGaugeWidget, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>


>,
    nullptr
} };


const QMetaObject *dac::TorqueGaugeWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::TorqueGaugeWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__TorqueGaugeWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int dac::TorqueGaugeWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 5;
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::BindableProperty
            || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
