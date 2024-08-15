OSX drivers for FTDI
====================

Drivers in this folder are installed by WICED Studio installer. 
To manually install the drivers, run the following commands under sudo -

> cd /System/Library/Extensions
> cp -R WicedAppleUSBFTDI.kext .
> chmod 755 ./WicedAppleUSBFTDI.kext 
> chown root:wheel ./WicedAppleUSBFTDI.kext 
> rm -R Extensions.kextcache 
> rm -R Extensions.mkext

For OSX version 10.10 or below only:
> cd /System/Library/Extensions
> cp -R WicedFTDIUSBSerialDriver.kext . 
> sudo chmod 755 ./FTDIUSBSerialDriver.kext
> chown root:wheel ./FTDIUSBSerialDriver.kext
> rm -R Extensions.kextcache 
> rm -R Extensions.mkext

