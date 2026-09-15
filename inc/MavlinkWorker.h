
#ifndef MAVLINKWORKER_H
#define MAVLINKWORKER_H

#include <QObject>
#include <QSerialPort>
#include <QString>

#include <ardupilotmega/mavlink.h>


class Mavlink : public QObject
{
    Q_OBJECT
public:
    explicit Mavlink(QObject *parent = nullptr);
    ~Mavlink();

    bool openPort(const QString &portName, int baudRate);
    bool closePort();
signals:
    void heartbeatReceived(int flightMode);
    void attitudeReceived(float roll, float pitch, float yaw);
    void parameterReceived(const QString &paramId, float value);

public slots:
    void requestParameterList();
    void setParameter(const QString &paramId, float value);

private slots:
    void readData();

private:
    QSerialPort *m_serialPort;
    QString m_portName;
    
    void parseMessage(const mavlink_message_t &msg);
};
#endif // MAVLINKWORKER_H