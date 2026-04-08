#include "models/MotorTableModel.h"

#include <QBrush>
#include <QColor>

MotorTableModel::MotorTableModel(QObject *parent)
    : QAbstractTableModel(parent)
{
}

int MotorTableModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid()) {
        return 0;
    }
    return m_configs.size();
}

int MotorTableModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid()) {
        return 0;
    }
    return ColumnCount;
}

QVariant MotorTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_configs.size()) {
        return {};
    }

    const MotorConfig &config = m_configs.at(index.row());
    const MotorState state = m_states.value(config.nodeId);

    if (role == Qt::DisplayRole) {
        switch (index.column()) {
        case NodeIdColumn:
            return formatCanId(config.nodeId);
        case NameColumn:
            return config.name;
        case OnlineColumn:
            return state.online ? QStringLiteral("在线") : QStringLiteral("离线");
        case EnabledColumn:
            return state.enabled ? QStringLiteral("已使能") : QStringLiteral("未使能");
        case ModeColumn:
            return state.modeText;
        case VelocityColumn:
            return QString::number(state.actualVelocityRpm);
        case PositionColumn:
            return QString::number(state.actualPositionDeg, 'f', 2);
        case TorqueColumn:
            return QString::number(state.torquePermille);
        case CurrentColumn:
            return QString::number(state.currentAmp, 'f', 2);
        case FaultColumn:
            if (state.faultCode == 0) {
                return QStringLiteral("-");
            }
            return QStringLiteral("0x%1 %2")
                .arg(state.faultCode, 4, 16, QLatin1Char('0'))
                .arg(state.faultText);
        case CollisionColumn:
            return state.collisionTriggered ? QStringLiteral("已触发") : QStringLiteral("正常");
        case LastUpdateColumn:
            return state.lastUpdate.isValid() ? state.lastUpdate.toString(QStringLiteral("HH:mm:ss.zzz")) : QStringLiteral("-");
        default:
            return {};
        }
    }

    if (role == Qt::TextAlignmentRole) {
        return Qt::AlignCenter;
    }

    if (role == Qt::ForegroundRole) {
        if (index.column() == OnlineColumn) {
            return QBrush(state.online ? QColor("#0f766e") : QColor("#b91c1c"));
        }
        if (index.column() == CollisionColumn && state.collisionTriggered) {
            return QBrush(QColor("#b45309"));
        }
        if (index.column() == FaultColumn && state.faultCode != 0) {
            return QBrush(QColor("#b91c1c"));
        }
    }

    return {};
}

QVariant MotorTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
        return QAbstractTableModel::headerData(section, orientation, role);
    }

    switch (section) {
    case NodeIdColumn:
        return QStringLiteral("节点");
    case NameColumn:
        return QStringLiteral("名称");
    case OnlineColumn:
        return QStringLiteral("在线");
    case EnabledColumn:
        return QStringLiteral("使能");
    case ModeColumn:
        return QStringLiteral("模式");
    case VelocityColumn:
        return QStringLiteral("速度 RPM");
    case PositionColumn:
        return QStringLiteral("位置 角度");
    case TorqueColumn:
        return QStringLiteral("转矩 ‰");
    case CurrentColumn:
        return QStringLiteral("电流 A");
    case FaultColumn:
        return QStringLiteral("故障");
    case CollisionColumn:
        return QStringLiteral("碰撞保护");
    case LastUpdateColumn:
        return QStringLiteral("最后更新");
    default:
        return {};
    }
}

void MotorTableModel::setMotorConfigs(const QList<MotorConfig> &configs)
{
    beginResetModel();
    m_configs = configs;
    m_states.clear();
    for (const MotorConfig &config : m_configs) {
        MotorState state;
        state.nodeId = config.nodeId;
        state.name = config.name;
        m_states.insert(config.nodeId, state);
    }
    endResetModel();
}

void MotorTableModel::updateState(const MotorState &state)
{
    const int row = rowForNode(state.nodeId);
    if (row < 0) {
        return;
    }

    m_states.insert(state.nodeId, state);
    const QModelIndex left = index(row, 0);
    const QModelIndex right = index(row, ColumnCount - 1);
    emit dataChanged(left, right);
}

void MotorTableModel::setAllOffline()
{
    for (const MotorConfig &config : m_configs) {
        MotorState state = m_states.value(config.nodeId);
        state.online = false;
        state.enabled = false;
        state.lastUpdate = {};
        m_states.insert(config.nodeId, state);
    }

    if (!m_configs.isEmpty()) {
        emit dataChanged(index(0, 0), index(m_configs.size() - 1, ColumnCount - 1));
    }
}

quint32 MotorTableModel::nodeIdAtRow(int row) const
{
    if (row < 0 || row >= m_configs.size()) {
        return 0;
    }
    return m_configs.at(row).nodeId;
}

int MotorTableModel::rowForNode(quint32 nodeId) const
{
    for (int row = 0; row < m_configs.size(); ++row) {
        if (m_configs.at(row).nodeId == nodeId) {
            return row;
        }
    }
    return -1;
}
