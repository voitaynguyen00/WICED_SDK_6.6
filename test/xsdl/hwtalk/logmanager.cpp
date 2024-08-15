#include "logmanager.h"
#include "osobjects.h"

#include <sstream>
#include <iomanip>

LogManager::LogManager()
{
    m_pLogFile = NULL;
#ifdef QT_APP
    m_pDisplay = NULL;
#endif
}

bool LogManager::OpenLogFile(string sLogFile)
{
    if (!(m_pLogFile = fopen(sLogFile.c_str(), "w")))
        return false;

    return true;
}

void LogManager::CloseLogFile()
{
    if (m_pLogFile)
    {
        fflush(m_pLogFile);
        fclose(m_pLogFile);
        m_pLogFile = NULL;
    }
}

void LogManager::WriteLog(string sLog)
{
    if (m_pLogFile)
    {
#if 0
        char buf[512];
#endif
        string sFullTrace;
        DATETIMEHOLDER oTime;

        GetCurrentOSTime(&oTime);

#if 1
	    std::ostringstream ss;
	    ss << std::setw(2) << std::setfill('0') << (unsigned short)oTime.m_hh << ":"
	       << std::setw(2) << std::setfill('0') << (unsigned short)oTime.m_mm << ":"
	       << std::setw(2) << std::setfill('0') << (unsigned short)oTime.m_ss << "."
	       << std::setw(3) << std::setfill('0') << (unsigned short)oTime.m_ms << "    " 
	       << sLog;
	    sFullTrace.assign(ss.str());
#else
        sprintf(buf, "%02d:%02d:%02d.%03d    %s", oTime.m_hh, oTime.m_mm, oTime.m_ss, oTime.m_ms, sLog.c_str());
        sFullTrace = buf;
#endif

        if (m_pLogFile)
        {
            fprintf(m_pLogFile, "%s\n", sFullTrace.c_str());
        }
#ifdef QT_APP
        if (m_pDisplay)
        {
            QListWidgetItem* pTraceItem = new QListWidgetItem();
            pTraceItem->setText(QString::fromStdString(sFullTrace));
            m_pDisplay->addItem(pTraceItem);
        }
#endif
    }
}

void LogManager::WriteRcvData(string sRcvHexData)
{
    if (m_pLogFile)
    {
        if (!sRcvHexData.empty())
        {
            string sLog = "Received:  Hex_Data={ " + sRcvHexData + " }";
            WriteLog("----------------------------------------------------");
            WriteLog(sLog);
        }
    }
}

void LogManager::WriteSendData(string sSendHexData)
{
    if (m_pLogFile)
    {
        if (!sSendHexData.empty())
        {
            string sLog = "Sent:  Hex_Data={ " + sSendHexData + " }";
            WriteLog("----------------------------------------------------");
            WriteLog(sLog);
        }
    }
}

void LogManager::WriteInstanceInfo(string sName)
{
    if (m_pLogFile)
    {
        string sLog = "Name: " + sName;
        WriteLog(sLog);
    }
}

void LogManager::WriteFieldInfo(string sName, string sValue)
{
    if (m_pLogFile)
    {
        string sLog = "      " + sName + " = " + sValue;
        WriteLog(sLog);
    }
}

#ifdef QT_APP
void LogManager::SetDisplayWidget(QListWidget *pWidget)
{
    m_pDisplay = pWidget;
}
#endif
