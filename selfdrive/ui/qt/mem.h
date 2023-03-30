#pragma once
#include <QString>

int readMemInt(QString param, int defaultVal, bool persistant);
double readMemDouble(QString param, double defaultVal, bool persistant);
QString readMemString(QString param, QString defaultVal, bool persistant);
bool readMemBool(QString param, bool defaultVal, bool persistant);

void writeMemBool(QString param, bool val, bool persistant);
void writeMemInt(QString param, int val, bool persistant);
void writeMemDouble(QString param, double val, bool persistant);
void writeMemString(QString param, QString val, bool persistant);
