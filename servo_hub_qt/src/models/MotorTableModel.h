#pragma once

#include "models/Types.h"

#include <QAbstractTableModel>
#include <QHash>
#include <QList>

class MotorTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum Column {
        NodeIdColumn = 0,
        NameColumn,
        OnlineColumn,
        EnabledColumn,
        ModeColumn,
        VelocityColumn,
        PositionColumn,
        TorqueColumn,
        CurrentColumn,
        FaultColumn,
        CollisionColumn,
        LastUpdateColumn,
        ColumnCount
    };

    explicit MotorTableModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    void setMotorConfigs(const QList<MotorConfig> &configs);
    void updateState(const MotorState &state);
    void setAllOffline();
    quint32 nodeIdAtRow(int row) const;

private:
    int rowForNode(quint32 nodeId) const;

    QList<MotorConfig> m_configs;
    QHash<quint32, MotorState> m_states;
};

