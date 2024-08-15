#ifndef LOGMANAGER_H
#define LOGMANAGER_H

#ifdef QT_APP
#include <QListWidget>
#endif

#include <string>

using namespace std;

class LogManager
{
public:
    LogManager();

    bool OpenLogFile(string sLogFile);
    void CloseLogFile();  
    void WriteLog(string sLog);
    void WriteRcvData(string sRcvHexData);
    void WriteSendData(string sSendHexData);
    void WriteInstanceInfo(string sName);
    void WriteFieldInfo(string sName, string sValue);




#ifdef QT_APP
    void SetDisplayWidget(QListWidget* pWidget);
#endif

private:
    FILE*           m_pLogFile;

#ifdef QT_APP
    QListWidget*    m_pDisplay;
#endif
};

#endif // LOGMANAGER_H
