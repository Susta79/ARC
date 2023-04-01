/****************************************************************************
** Meta object code from reading C++ file 'robot.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/robot.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'robot.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Robot_t {
    QByteArrayData data[13];
    char stringdata0[96];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Robot_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Robot_t qt_meta_stringdata_Robot = {
    {
QT_MOC_LITERAL(0, 0, 5), // "Robot"
QT_MOC_LITERAL(1, 6, 13), // "pbFK_released"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 13), // "pbIK_released"
QT_MOC_LITERAL(4, 35, 9), // "FrontBack"
QT_MOC_LITERAL(5, 45, 5), // "Front"
QT_MOC_LITERAL(6, 51, 4), // "Back"
QT_MOC_LITERAL(7, 56, 6), // "UpDown"
QT_MOC_LITERAL(8, 63, 2), // "Up"
QT_MOC_LITERAL(9, 66, 4), // "Down"
QT_MOC_LITERAL(10, 71, 6), // "PosNeg"
QT_MOC_LITERAL(11, 78, 8), // "Positive"
QT_MOC_LITERAL(12, 87, 8) // "Negative"

    },
    "Robot\0pbFK_released\0\0pbIK_released\0"
    "FrontBack\0Front\0Back\0UpDown\0Up\0Down\0"
    "PosNeg\0Positive\0Negative"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Robot[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       3,   26, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x08 /* Private */,
       3,    0,   25,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

 // enums: name, alias, flags, count, data
       4,    4, 0x0,    2,   41,
       7,    7, 0x0,    2,   45,
      10,   10, 0x0,    2,   49,

 // enum data: key, value
       5, uint(Robot::Front),
       6, uint(Robot::Back),
       8, uint(Robot::Up),
       9, uint(Robot::Down),
      11, uint(Robot::Positive),
      12, uint(Robot::Negative),

       0        // eod
};

void Robot::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Robot *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->pbFK_released(); break;
        case 1: _t->pbIK_released(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject Robot::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Robot.data,
    qt_meta_data_Robot,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Robot::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Robot::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Robot.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Robot::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
