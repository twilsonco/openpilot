#include <QJsonDocument>
#include <QTextStream>
#include <QVariant>
#include <QJsonParseError>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>
#include <QFile>
#include <QString>
#include <QThread>

void waitMemLock(QString param) {
  QString path = QString("/dev/shm/_c_mem_") + param + QString(".lock");
  QFile file(path);
  while(!file.open(QIODevice::NewOnly)) {
    QThread::msleep(1);
  };
}

void memUnlock(QString param) {
  QString path = QString("/dev/shm/_c_mem_") + param + QString(".lock");
  QFile file(path);
  file.remove();

}

int readMemInt(QString param, int defaultVal, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  if(persistant) {
    path = QString("/data/params/_c_mem_") + param;
  }
  QFile file(path);
  if(!file.open(QIODevice::ReadOnly)) {
    memUnlock(param);
    return defaultVal;
  };

  QString val = file.readAll();
  file.close();
  memUnlock(param);

  QJsonDocument d = QJsonDocument::fromJson(val.toUtf8());
  return d[0].toInt();
}

double readMemDouble(QString param, double defaultVal, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  if(persistant) {
    path = QString("/data/params/_c_mem_") + param;
  }
  QFile file(path);
  if(!file.open(QIODevice::ReadOnly)) {
    memUnlock(param);
    return defaultVal;
  };

  QString val = file.readAll();
  file.close();
  memUnlock(param);

  QJsonDocument d = QJsonDocument::fromJson(val.toUtf8());
  return d[0].toDouble();
}


QString readMemString(QString param, QString defaultVal, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  if(persistant) {
    path = QString("/data/params/_c_mem_") + param;
  }
  QFile file(path);
  if(!file.open(QIODevice::ReadOnly)) {
    memUnlock(param);
    return defaultVal;
  };

  QString val = file.readAll();
  file.close();
  memUnlock(param);

  QJsonDocument d = QJsonDocument::fromJson(val.toUtf8());
  return d[0].toString();
}

bool readMemBool(QString param, bool defaultVal, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  if(persistant) {
    path = QString("/data/params/_c_mem_") + param;
  }
  QFile file(path);
  if(!file.open(QIODevice::ReadOnly)) {
    memUnlock(param);
    return defaultVal;
  };

  QString val = file.readAll();
  file.close();
  memUnlock(param);

  QJsonDocument d = QJsonDocument::fromJson(val.toUtf8());
  return d[0].toBool();
}

bool writeJsonVal(QString path, QJsonValue val) {
  QJsonArray arr = QJsonArray();
  arr.append(val);
  QJsonDocument doc(arr);

  QFile file(path);
  if(!file.open(QIODevice::WriteOnly)) {
    return false;
  };

  QTextStream out(&file);
  out << doc.toJson();
  file.close();
  return true;
}

void writeMemInt(QString param, int val, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  QString persist_path = QString("/data/params/_c_mem_") + param;

  QJsonValue jval = QJsonValue(val);

  if(persistant) {
    if(!writeJsonVal(persist_path, jval)) {
      memUnlock(param);
      return;
    }
  }
  writeJsonVal(path, jval);
  memUnlock(param);
}

void writeMemDouble(QString param, double val, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  QString persist_path = QString("/data/params/_c_mem_") + param;

  QJsonValue jval = QJsonValue(val);

  if(persistant) {
    if(!writeJsonVal(persist_path, jval)) {
      memUnlock(param);
      return;
    }
  }
  writeJsonVal(path, jval);
  memUnlock(param);
}


void writeMemString(QString param, QString val, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  QString persist_path = QString("/data/params/_c_mem_") + param;

  QJsonValue jval = QJsonValue(val);

  if(persistant) {
    if(!writeJsonVal(persist_path, jval)) {
      memUnlock(param);
      return;
    }
  }
  writeJsonVal(path, jval);
  memUnlock(param);
}

void writeMemBool(QString param, bool val, bool persistant) {
  waitMemLock(param);
  QString path = QString("/dev/shm/_c_mem_") + param;
  QString persist_path = QString("/data/params/_c_mem_") + param;

  QJsonValue jval = QJsonValue(val);

  if(persistant) {
    if(!writeJsonVal(persist_path, jval)) {
      memUnlock(param);
      return;
    }
  }
  writeJsonVal(path, jval);
  memUnlock(param);
}
