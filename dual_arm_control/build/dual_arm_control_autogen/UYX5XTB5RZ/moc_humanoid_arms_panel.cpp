/****************************************************************************
** Meta object code from reading C++ file 'humanoid_arms_panel.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ui/humanoid_arms_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'humanoid_arms_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__HumanoidArmsPanel_t {
    const uint offsetsAndSize[30];
    char stringdata0[214];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__HumanoidArmsPanel_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__HumanoidArmsPanel_t qt_meta_stringdata_dac__HumanoidArmsPanel = {
    {
QT_MOC_LITERAL(0, 22), // "dac::HumanoidArmsPanel"
QT_MOC_LITERAL(23, 16), // "connectRequested"
QT_MOC_LITERAL(40, 0), // ""
QT_MOC_LITERAL(41, 11), // "deviceIndex"
QT_MOC_LITERAL(53, 8), // "canIndex"
QT_MOC_LITERAL(62, 19), // "disconnectRequested"
QT_MOC_LITERAL(82, 16), // "refreshRequested"
QT_MOC_LITERAL(99, 21), // "jointTargetsRequested"
QT_MOC_LITERAL(121, 12), // "dac::ArmSide"
QT_MOC_LITERAL(134, 4), // "side"
QT_MOC_LITERAL(139, 13), // "QList<double>"
QT_MOC_LITERAL(153, 15), // "jointTargetsDeg"
QT_MOC_LITERAL(169, 8), // "velocity"
QT_MOC_LITERAL(178, 14), // "brakeRequested"
QT_MOC_LITERAL(193, 20) // "clearErrorsRequested"

    },
    "dac::HumanoidArmsPanel\0connectRequested\0"
    "\0deviceIndex\0canIndex\0disconnectRequested\0"
    "refreshRequested\0jointTargetsRequested\0"
    "dac::ArmSide\0side\0QList<double>\0"
    "jointTargetsDeg\0velocity\0brakeRequested\0"
    "clearErrorsRequested"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__HumanoidArmsPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   50,    2, 0x06,    1 /* Public */,
       5,    0,   55,    2, 0x06,    4 /* Public */,
       6,    0,   56,    2, 0x06,    5 /* Public */,
       7,    3,   57,    2, 0x06,    6 /* Public */,
      13,    1,   64,    2, 0x06,   10 /* Public */,
      14,    1,   67,    2, 0x06,   12 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8, 0x80000000 | 10, QMetaType::Double,    9,   11,   12,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, 0x80000000 | 8,    9,

       0        // eod
};

void dac::HumanoidArmsPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<HumanoidArmsPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->connectRequested((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 1: _t->disconnectRequested(); break;
        case 2: _t->refreshRequested(); break;
        case 3: _t->jointTargetsRequested((*reinterpret_cast< std::add_pointer_t<dac::ArmSide>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QList<double>>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 4: _t->brakeRequested((*reinterpret_cast< std::add_pointer_t<dac::ArmSide>>(_a[1]))); break;
        case 5: _t->clearErrorsRequested((*reinterpret_cast< std::add_pointer_t<dac::ArmSide>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QList<double> >(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmSide >(); break;
            }
            break;
        case 4:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmSide >(); break;
            }
            break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmSide >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (HumanoidArmsPanel::*)(int , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsPanel::connectRequested)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (HumanoidArmsPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsPanel::disconnectRequested)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (HumanoidArmsPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsPanel::refreshRequested)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (HumanoidArmsPanel::*)(dac::ArmSide , const QVector<double> & , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsPanel::jointTargetsRequested)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (HumanoidArmsPanel::*)(dac::ArmSide );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsPanel::brakeRequested)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (HumanoidArmsPanel::*)(dac::ArmSide );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsPanel::clearErrorsRequested)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject dac::HumanoidArmsPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_dac__HumanoidArmsPanel.offsetsAndSize,
    qt_meta_data_dac__HumanoidArmsPanel,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__HumanoidArmsPanel_t
, QtPrivate::TypeAndForceComplete<HumanoidArmsPanel, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::ArmSide, std::false_type>, QtPrivate::TypeAndForceComplete<const QVector<double> &, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::ArmSide, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::ArmSide, std::false_type>



>,
    nullptr
} };


const QMetaObject *dac::HumanoidArmsPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::HumanoidArmsPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__HumanoidArmsPanel.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int dac::HumanoidArmsPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void dac::HumanoidArmsPanel::connectRequested(int _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::HumanoidArmsPanel::disconnectRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void dac::HumanoidArmsPanel::refreshRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void dac::HumanoidArmsPanel::jointTargetsRequested(dac::ArmSide _t1, const QVector<double> & _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void dac::HumanoidArmsPanel::brakeRequested(dac::ArmSide _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void dac::HumanoidArmsPanel::clearErrorsRequested(dac::ArmSide _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
