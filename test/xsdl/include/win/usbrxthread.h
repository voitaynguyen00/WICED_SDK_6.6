#ifndef USB_RX_THREAD_H_INCLUDED
#define USB_RX_THREAD_H_INCLUDED

extern "C" DWORD WINAPI USBEventReaderThreadProc(LPVOID lpV);
extern "C" DWORD WINAPI USBDataReaderThreadProc(LPVOID lpV);

#endif //USB_RX_THREAD_H_INCLUDED
