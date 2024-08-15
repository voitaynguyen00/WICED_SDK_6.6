/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * OTA upgrader process implementation.
 */

import Foundation

public enum OtaCharacteristic {
    case controlPointCharacteristic
    case dataCharacteristic
    case appInfoCharacteristic
}

public protocol OtaUpgraderProtocol {
    /*
     * The OTA adapter can call this interface prepare for OTA upgrade process, including
     * connect to the OTA device, discover and read all OTA GATT service and characteristics,
     * also try to read the App Info if supported.
     */
    func otaUpgradePrepare(for device: OtaDeviceProtocol) -> Int
    
    /*
     * The OTA adapter can call this interface start OTA upgrade process.
     */
    func otaUpgradeStart(for device: OtaDeviceProtocol, fwImage: Data) -> Int

    /*
     * The OTA adapter must call this interface when the connection state to the remote OTA device has changed.
     */
    func didUpdateConnectionState(isConnected: Bool, error: Error?)
    
    /*
     * The OTA adapter must call this interface when the OTA service and charactieristics discovering has completed.
     * After the OTA service has been discovered, the OTA adapter should save those instacen for later usage.
     */
    func didUpdateOtaServiceCharacteristicState(isDiscovered: Bool, error: Error?)
    
    /*
     * The OTA adapter must call this interface when the notification states of the OTA service Control Pointer characteristic is udpated,
     */
    func didUpdateNotificationState(isEnabled: Bool, error: Error?)
    
    /*
     * The OTA adapter must call this interface when any data received from any OTA service characteristic,
     * inlcuding the characterisitic indication/notification data and read value data.
     */
    func didUpdateValueFor(characteristic: OtaCharacteristic, value: Data?, error: Error?)
}

open class OtaUpgrader: OtaUpgraderProtocol {
    public static let shared = OtaUpgrader()
    
    private var otaDevice: OtaDeviceProtocol?
    open var delegate: OtaDeviceProtocol? {
        get {
            return otaDevice
        }
        set {
            otaDevice = delegate
        }
    }
    
    private var otaCommandTimer: Timer?
    private let lock = NSLock()
    public var isOtaUpgradeRunning: Bool = false
    public var isDeviceConnected: Bool = false
    public var isOtaUpgradePrepareReady: Bool = false
    public var prepareOtaUpgradeOnly: Bool = false
    
    private var fwImage: Data?
    private var fwImageSize: Int = 0
    private var fwOffset: Int = 0
    private var transferringSize: Int = 0
    private var fwCrc32 = OtaUpgrader.CRC32_INIT_VALUE
    private var maxOtaPacketSize: Int = 155
    
    private var state: OtaState = .idle
    private var completeError: OtaError?
    
    private var isGetComponentInfoRunning: Bool = false
    
    open func otaUpgradeStatusReset() {
        lock.lock()
        isGetComponentInfoRunning = false
        isOtaUpgradeRunning = false
        isDeviceConnected = false
        isOtaUpgradePrepareReady = false
        prepareOtaUpgradeOnly = false
        completeError = nil
        state = .idle
        lock.unlock()
    }
    
    open func dumpOtaUpgradeStatus() {
        print("dumpOtaUpgradeStatus, otaState:\(state.description), isOtaUpgradeRunning:\(isOtaUpgradeRunning), isDeviceConnected:\(isDeviceConnected), prepareOtaUpgradeOnly:\(prepareOtaUpgradeOnly), isOtaUpgradePrepareReady:\(isOtaUpgradePrepareReady)")
    }
    
    open func otaUpgradePrepare(for device: OtaDeviceProtocol) -> Int {
        lock.lock()
        if isOtaUpgradeRunning {
            dumpOtaUpgradeStatus()
            print("error: OtaUpgrader, otaUpgradeStart, ota upgrader has been started, busying")
            lock.unlock()
            return OtaErrorCode.BUSYING
        }
        isGetComponentInfoRunning = false
        isOtaUpgradeRunning = true
        isDeviceConnected = false
        prepareOtaUpgradeOnly = true
        isOtaUpgradePrepareReady = false
        lock.unlock()
        
        self.state = .idle
        self.otaDevice = device
        self.completeError = nil
        OtaNotificationData.init(otaError: OtaError(state: .idle, code: OtaErrorCode.SUCCESS, desc: "otaUpgradePrepare started")).post()
        DispatchQueue.main.async {
            self.stateMachineProcess()
        }
        return OtaErrorCode.SUCCESS
    }
    
    open func otaUpgradeStart(for device: OtaDeviceProtocol, fwImage: Data) -> Int {
        lock.lock()
        if !isOtaUpgradePrepareReady, isOtaUpgradeRunning {
            dumpOtaUpgradeStatus()
            print("error: OtaUpgrader, otaUpgradeStart, ota upgrader has been started, busying")
            lock.unlock()
            return OtaErrorCode.BUSYING
        }
        isOtaUpgradeRunning = true
        lock.unlock()
        guard fwImage.count > 0 else {
            print("error: OtaUpgrader, otaUpgradeStart, invalid OTA firmware image size: \(fwImage.count)")
            lock.lock()
            isOtaUpgradeRunning = false
            lock.unlock()
            return OtaErrorCode.INVALID_FW_IMAGE
        }
        
        if isOtaUpgradePrepareReady, isDeviceConnected,
            let preparedDevice = self.otaDevice, preparedDevice.equal(device),
            preparedDevice.otaDevice != nil, preparedDevice.otaService != nil,
            preparedDevice.otaControlPointCharacteristic != nil, preparedDevice.otaDataCharacteristic != nil {
            self.state = .enableNotification
        } else {
            self.otaDevice = device
            self.state = .idle
        }
        prepareOtaUpgradeOnly = false
        isGetComponentInfoRunning = false
        
        self.fwImage = fwImage
        self.fwImageSize = fwImage.count
        self.fwOffset = 0
        self.transferringSize = 0
        self.fwCrc32 = OtaUpgrader.CRC32_INIT_VALUE
        self.maxOtaPacketSize = OtaUpgrader.getMaxDataTransferSize(deviceType: device.getDeviceType())
        self.completeError = nil
        
        print("OtaUpgrader, otaUpgradeStart, otaDevice name:\(device.getDeviceName()), type:\(device.getDeviceType())")
        OtaNotificationData.init(otaError: OtaError(state: .idle, code: OtaErrorCode.SUCCESS, desc: "otaUpgradeStart started")).post()
        DispatchQueue.main.async {
            self.stateMachineProcess()
        }
        
        // Now, OTA upgrade processing has been started, progress status will be updated through OtaConstants.Notification.OTA_COMPLETE_STATUS notificaitons.
        return OtaErrorCode.SUCCESS
    }
    
    ///
    /// Interfaces for receiving notification or response data from remote device.
    ///
    
    open func didUpdateConnectionState(isConnected: Bool, error: Error?) {
        guard isOtaUpgradeRunning, state != .idle else {
            return
        }
        
        /*
         * some old device, there no response data for the verify command, and the device will reset itself after about 2 seconds when verify succes,
         * when verify failed, the verify response with failure status will be received immeidately.
         * so, here process the disconnection event as verify success and the upgrade process has been successfully done in firmware side.
         */
        if state == .verify,  otaCommandTimer?.isValid ?? false, !isConnected {
            otaVerifyResponse(data: Data(repeating: 0, count: 1), error: nil)
            return
        }
        
        stopOtaCommandTimer()
        guard error == nil, isConnected else {
            if error != nil {
                print("error: OtaUpgrader, didUpdateConnectionState, unexpected disconnect or failed to connect to remote OTA device, error:\(error!)")
                completeError = OtaError(state: state, code: OtaErrorCode.ERROR_DEVICE_CONNECT, desc: "failed to connect to remote device")
            } else {
                print("error: OtaUpgrader, didUpdateConnectionState, disconnected from remote OTA device")
                completeError = OtaError(state: state, code: OtaErrorCode.ERROR_DEVICE_DISCONNECT, desc: "disconnect from remote device")
            }
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }

        OtaNotificationData(otaState: state, otaError: nil).post()
        state = .otaServiceDiscover
        stateMachineProcess()
    }
    
    /*
     * The OTA adapter must call this interface when the OTA service and charactieristics discovering has completed.
     * After the OTA service has been discovered, the OTA adapter should save those instacen for later usage.
     */
    open func didUpdateOtaServiceCharacteristicState(isDiscovered: Bool, error: Error?) {
        guard isOtaUpgradeRunning, isDeviceConnected, state != .idle else {
            return
        }
        
        stopOtaCommandTimer()
        guard error == nil, isDiscovered else {
            OtaManager.shared.dumpOtaStatus()
            if error != nil {
                print("error: OtaUpgrader, didUpdateOtaServiceCharacteristicState, failed to discover OTA GATT service, error:\(error!)")
                completeError = OtaError(state: state, code: OtaErrorCode.ERROR_DISCOVER_SERVICE, desc: "discover OTA service with error")
            } else {
                print("error: OtaUpgrader, didUpdateOtaServiceCharacteristicState, no OTA GATT service discovered from remote OTA device")
                completeError = OtaError(state: state, code: OtaErrorCode.ERROR_DEVICE_OTA_NOT_SUPPORTED, desc: "no OTA service discovered")
            }
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }

        OtaNotificationData(otaState: state, otaError: nil).post()
        if (otaDevice?.otaAppInfoCharacteristic != nil) || (self.otaDevice != nil && self.otaDevice!.getDeviceType() == .mesh) {
            state = .readAppInfo
        } else {
            if prepareOtaUpgradeOnly {
                self.otaReadAppInfoResponse(data: nil, error: nil)
                return
            }
            
            state = .enableNotification
        }
        stateMachineProcess()
    }
    
    /*
     * The OTA adapter must call this interface when the notification states of the OTA service Control Pointer characteristic is udpated,
     */
    open func didUpdateNotificationState(isEnabled: Bool, error: Error?) {
        guard isOtaUpgradeRunning, isDeviceConnected, state != .idle else {
            return
        }
        
        stopOtaCommandTimer()
        guard error == nil, isEnabled else {
            OtaManager.shared.dumpOtaStatus()
            if error != nil {
                print("error: OtaUpgrader, didUpdateNotificationState, failed to enable OTA Control Point characteristic notification, error:\(error!)")
                completeError = OtaError(state: state, code: OtaErrorCode.ERROR_CHARACTERISTIC_NOTIFICATION_UPDATE, desc: "enable notification with error")
            } else {
                print("error: OtaUpgrader, didUpdateOtaServiceCharacteristicState, OTA Control Point characteristic notification not enabled")
                completeError = OtaError(state: state, code: OtaErrorCode.ERROR_CHARACTERISTIC_NOTIFICATION_UPDATE, desc: "notification disabled")
            }
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        OtaNotificationData(otaState: state, otaError: nil).post()
        state = .prepareForDownload
        stateMachineProcess()
    }
    
    /*
     * The OTA adapter must call this interface when any data received from any OTA service characteristic,
     * inlcuding the characterisitic indication/notification data and read value data.
     */
    open func didUpdateValueFor(characteristic: OtaCharacteristic, value: Data?, error: Error?) {
        guard isOtaUpgradeRunning, isDeviceConnected, state != .idle else {
            return
        }
        
        switch state {
        case .readAppInfo:
            otaReadAppInfoResponse(data: value, error: error)
        case .prepareForDownload:
            otaPrepareForDownloadResponse(data: value, error: error)
        case .startDownload:
            otaStartDownloadResponse(data: value, error: error)
        case .dataTransfer:
            otaTransferDataResponse(data: value, error: error)
        case .verify:
            otaVerifyResponse(data: value, error: error)
        case .abort:
            otaAbortResponse(data: value, error: error)
        default:
            break
        }
    }
    
    private func stateMachineProcess() {
        switch self.state {
        case .idle:
            isOtaUpgradeRunning = true
            self.state = .connect
            self.stateMachineProcess()
        case .connect:
            self.otaConnect()
        case .otaServiceDiscover:
            isDeviceConnected = true
            self.discoverOtaServiceCharacteristics()
        case .readAppInfo:
            self.otaReadAppInfo()
        case .enableNotification:
            self.otaEnableNotification()
        case .prepareForDownload:
            self.otaPrepareForDownload()
        case .startDownload:
            self.otaStartDownload()
        case .dataTransfer:
            self.otaTransferData()
        case .verify:
            self.otaVerify()
        case .abort:
            self.otaAbort()
        case .complete:
            self.otaCompleted()
            lock.lock()
            if prepareOtaUpgradeOnly {
                // Do not clear the isOtaUpgradeRunning and isDeviceConnected state values
                // when only do prepare for OTA upgrade, because mesh network may change the connection status,
                // so, must keep the isOtaUpgrading to track the changes to avoid any potential incnsistent issue.
            } else {
                isDeviceConnected = false
            }
            isOtaUpgradeRunning = false
            isGetComponentInfoRunning = false
            lock.unlock()
            print("OtaUpgrader, stateMachineProcess, exit")
        }
    }
    
    private func otaConnect() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaConnect, otaDevice instance is nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        startOtaCommandTimer()
        otaDevice.connect()
    }
    
    private func discoverOtaServiceCharacteristics() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, discoverOtaServiceCharacteristics, invalid delegate:nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        startOtaCommandTimer()
        otaDevice.discoverOtaServiceCharacteristic()
    }
    
    private func otaReadAppInfo() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaReadAppInfo, otaDevice instance is nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        if otaDevice.otaAppInfoCharacteristic != nil {
            startOtaCommandTimer()
            otaDevice.readValue(from: .appInfoCharacteristic)
        } else {
            self.otaReadAppInfoResponse(data: nil, error: nil)
        }
    }
    
    private func otaReadAppInfoResponse(data: Data?, error: Error?) {
        stopOtaCommandTimer()
        
        // alawys ignore the readAppInfo error status, because many device doesn't support AppInfo characateristic at all.
        if let appInfoData = data {
            if appInfoData.count == 4 {
                let appId  = UInt16(UInt8(appInfoData[0])) + (UInt16(UInt8(appInfoData[1])) << 8)
                let appVerMajor = UInt8(appInfoData[2])
                let appVerMinor = UInt8(appInfoData[3])
                let appInfoString = String(format: "AppId: 0x%04X, AppVersion: %d.%d",
                                           appId, appVerMajor, appVerMinor)
                OtaNotificationData(otaError: OtaError(state: state, code: OtaErrorCode.SUCCESS, desc: appInfoString)).post()
            } else if appInfoData.count == 5 {
                let appId  = UInt16(UInt8(appInfoData[0])) + (UInt16(UInt8(appInfoData[1])) << 8)
                let appVerPrefixNumber = UInt8(appInfoData[2])
                let appVerMajor = UInt8(appInfoData[3])
                let appVerMinor = UInt8(appInfoData[4])
                let appInfoString = String(format: "AppId: 0x%04X, AppVersion: %d.%d.%d",
                                           appId, appVerPrefixNumber, appVerMajor, appVerMinor)
                OtaNotificationData(otaError: OtaError(state: state, code: OtaErrorCode.SUCCESS, desc: appInfoString)).post()
            }
        } else {
            if let otaDevice = self.otaDevice, otaDevice.getDeviceType() == .mesh {
                if self.isGetComponentInfoRunning, self.otaCommandTimer?.isValid ?? false {
                    return  // avoid the getComponentInfo command send multiple times.
                }
                
                if !MeshFrameworkManager.shared.isMeshNetworkConnected() {
                    if self.prepareOtaUpgradeOnly {
                        self.state = .complete
                    } else {
                        self.state = .enableNotification
                    }
                    self.isOtaUpgradePrepareReady = true
                    self.stateMachineProcess()
                    return
                }
                
                startOtaCommandTimer()
                self.isGetComponentInfoRunning = true
                MeshFrameworkManager.shared.getMeshComponentInfo(componentName: otaDevice.getDeviceName()) { (componentName: String, componentInfo: String?, error: Int) in
                    self.isGetComponentInfoRunning = false
                    self.stopOtaCommandTimer()
                    if error == MeshErrorCode.MESH_SUCCESS, let componentInfo = componentInfo {
                        OtaNotificationData(otaError: OtaError(state: self.state, code: OtaErrorCode.SUCCESS, desc: componentInfo)).post()
                    }
                    
                    if self.prepareOtaUpgradeOnly {
                        self.state = .complete
                    } else {
                        self.state = .enableNotification
                    }
                    self.isOtaUpgradePrepareReady = true
                    self.stateMachineProcess()
                }
                return
            }
        }

        if prepareOtaUpgradeOnly {
            state = .complete
        } else {
            state = .enableNotification
        }
        isOtaUpgradePrepareReady = true
        stateMachineProcess()
    }
    
    private func otaEnableNotification() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaEnableNotification, otaDevice instance is nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        startOtaCommandTimer()
        otaDevice.enableOtaNotification(enabled: true)
    }
    
    private func otaPrepareForDownload() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaPrepareForDownload, otaDevice instance is nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        startOtaCommandTimer()
        let otaCommand = OtaCommandData(command: .prepareDownload)
        otaDevice.writeValue(to: .controlPointCharacteristic, value: otaCommand.value) { (data, error) in
            guard error == nil else {
                OtaManager.shared.dumpOtaStatus()
                print("error: OtaUpgrader, otaPrepareForDownload, failed to write PrepareForDownload command, error:\(String(describing: error))")
                self.completeError = OtaError(state: self.state,
                                              code: OtaErrorCode.ERROR_CHARACTERISTIC_WRITE_VALUE,
                                              desc: "failed to write PrepareForDownload command")
                OtaNotificationData(otaError: self.completeError!).post()
                self.state = .complete
                self.stateMachineProcess()
                return
            }
        }
    }
    
    private func otaPrepareForDownloadResponse(data: Data?, error: Error?) {
        stopOtaCommandTimer()
        guard error == nil else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaPrepareForDownload response with error:\(error!)")
            completeError = OtaError(state: state, code: OtaErrorCode.FAILED, desc: "failed to send prepre for download command")
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        let status = OtaCommandStatus.parse(from: data)
        print("OtaUpgrader, otaPrepareForDownloadResponse, status:\(status.description())")
        if status == .success {
            OtaNotificationData(otaState: state, otaError: nil).post()
            state = .startDownload
        } else {
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_RESPONSE_VALUE, desc: "ota prepare for download response with failure")
            OtaNotificationData(otaState: state, otaError: completeError).post()
            state = .complete
        }
        stateMachineProcess()
    }
    
    private func otaStartDownload() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaStartDownload, otaDevice instance is nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .abort
            stateMachineProcess()
            return
        }
        
        startOtaCommandTimer()
        let otaCommand = OtaCommandData(command: .startDownload, lParam: UInt32(fwImageSize))
        otaDevice.writeValue(to: .controlPointCharacteristic, value: otaCommand.value) { (data, error) in
            guard error == nil else {
                OtaManager.shared.dumpOtaStatus()
                print("error: OtaUpgrader, otaStartDownload, failed to write StartDownload command, error:\(String(describing: error))")
                self.completeError = OtaError(state: self.state,
                                              code: OtaErrorCode.ERROR_CHARACTERISTIC_WRITE_VALUE,
                                              desc: "failed to write StartDownload command")
                OtaNotificationData(otaError: self.completeError!).post()
                self.state = .abort
                self.stateMachineProcess()
                return
            }
        }
    }
    
    private func otaStartDownloadResponse(data: Data?, error: Error?) {
        stopOtaCommandTimer()
        guard error == nil else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaStartDownload response with error:\(error!)")
            completeError = OtaError(state: state, code: OtaErrorCode.FAILED, desc: "failed to send start download command")
            OtaNotificationData(otaError: completeError!).post()
            state = .abort
            stateMachineProcess()
            return
        }
        
        let status = OtaCommandStatus.parse(from: data)
        print("OtaUpgrader, otaStartDownloadResponse, status:\(status.description())")
        if status == .success {
            OtaNotificationData(otaState: state, otaError: nil).post()
            state = .dataTransfer
        } else {
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_RESPONSE_VALUE, desc: "ota start download response with failure")
            OtaNotificationData(otaState: state, otaError: completeError).post()
            state = .abort
        }
        stateMachineProcess()
    }
    
    private func otaTransferData() {
        guard let otaDevice = self.otaDevice, let fwImage = self.fwImage else {
            OtaManager.shared.dumpOtaStatus()
            if self.otaDevice == nil {
                print("error: OtaUpgrader, otaTransferData, otaDevice instance is nil")
                completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            } else {
                print("error: OtaUpgrader, otaTransferData, invalid fwImage nil")
                completeError = OtaError(state: state, code: OtaErrorCode.INVALID_FW_IMAGE, desc: "fw image data is nil")
            }
            OtaNotificationData(otaError: completeError!, fwImageSize: fwImageSize, transferredImageSize: fwOffset).post()
            state = .abort
            stateMachineProcess()
            return
        }
        
        if fwOffset == 0 {
            // send notifcation that indicate tranferring started.
            OtaNotificationData(otaState: state, otaError: nil, fwImageSize: fwImageSize, transferredImageSize: fwOffset).post()
        }

        let transferSize = fwImageSize - fwOffset
        transferringSize = (transferSize > maxOtaPacketSize) ? maxOtaPacketSize : transferSize
        if transferringSize > 0 {
            let range: Range = fwOffset..<(fwOffset + transferringSize)
            let transferData = fwImage.subdata(in: range)
            fwCrc32 = OtaUpgrader.calculateCrc32(crc32: fwCrc32, data: transferData)
            if (fwOffset + transferringSize) >= fwImageSize {
                fwCrc32 ^= OtaUpgrader.CRC32_INIT_VALUE     // this is the last packet, get final calculated fw image CRC value.
            }
            print("OtaUpgrader, otaTransferData, fwImageSize:\(fwImageSize), write at offset:\(fwOffset), size:\(transferringSize)")
            startOtaCommandTimer()
            otaDevice.writeValue(to: .dataCharacteristic, value: transferData, completion: self.otaTransferDataResponse)
        } else {
            print("warnning: OtaUpgrader, otaTransferData, no more data for transferring, fwImageSize:\(fwImageSize), offset:\(fwOffset)")
            stopOtaCommandTimer()
            fwOffset = fwImageSize
            OtaNotificationData(otaState: state, otaError: nil, fwImageSize: self.fwImageSize, transferredImageSize: fwOffset).post()
            
            state = .verify
            stateMachineProcess()
        }
    }
    
    private func otaTransferDataResponse(data: Data?, error: Error?) {
        stopOtaCommandTimer()
        guard error == nil else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaTransferData response with error:\(error!)")
            completeError = OtaError(state: state, code: OtaErrorCode.FAILED, desc: "failed to send out image data")
            OtaNotificationData(otaError: completeError!, fwImageSize: fwImageSize, transferredImageSize: fwOffset).post()
            state = .abort
            stateMachineProcess()
            return
        }
        
        fwOffset += transferringSize
        OtaNotificationData(otaState: state, otaError: nil, fwImageSize: self.fwImageSize, transferredImageSize: fwOffset).post()
        
        if fwOffset >= fwImageSize {
            print("OtaUpgrader, otaTransferData, fwImageSize:\(fwImageSize), totally transferred size:\(fwOffset), done")
            state = .verify
        }
        stateMachineProcess()
    }
    
    private func otaVerify() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaVerify, otaDevice instance is nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .abort
            stateMachineProcess()
            return
        }
        
        startOtaCommandTimer()
        let otaCommand = OtaCommandData(command: .verify, lParam: UInt32(fwCrc32))
        print("OtaUpgrader, otaVerify, CRC32=\(String.init(format: "0x%X", fwCrc32))")
        otaDevice.writeValue(to: .controlPointCharacteristic, value: otaCommand.value) { (data, error) in
            guard error == nil else {
                OtaManager.shared.dumpOtaStatus()
                print("error: OtaUpgrader, otaVerify, failed to write Verify command, error:\(String(describing: error))")
                self.completeError = OtaError(state: self.state,
                                              code: OtaErrorCode.ERROR_CHARACTERISTIC_WRITE_VALUE,
                                              desc: "failed to write Verify command")
                OtaNotificationData(otaError: self.completeError!).post()
                self.state = .abort
                self.stateMachineProcess()
                return
            }
        }
    }
    
    private func otaVerifyResponse(data: Data?, error: Error?) {
        stopOtaCommandTimer()
        guard error == nil else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaVerify response with error:\(error!)")
            completeError = OtaError(state: state, code: OtaErrorCode.FAILED, desc: "failed to send verify command")
            OtaNotificationData(otaError: completeError!).post()
            state = .abort
            stateMachineProcess()
            return
        }
        
        let status = OtaCommandStatus.parse(from: data)
        print("OtaUpgrader, otaVerifyResponse, status:\(status.description())")
        if status == .success {
            OtaNotificationData(otaState: state, otaError: nil).post()
            state = .complete
        } else {
            completeError = OtaError(state: state, code: OtaErrorCode.ERROR_OTA_VERIFICATION_FAILED, desc: "firmware downloaded image CRC32 verification failed")
            OtaNotificationData(otaState: state, otaError: completeError).post()
            state = .abort
        }
        stateMachineProcess()
    }
    
    private func otaAbort() {
        guard let otaDevice = self.otaDevice else {
            OtaManager.shared.dumpOtaStatus()
            print("error: OtaUpgrader, otaAbort, otaDevice instance is nil")
            completeError = OtaError(state: state, code: OtaErrorCode.INVALID_PARAMETERS, desc: "otaDevice instance is nil")
            OtaNotificationData(otaError: completeError!).post()
            state = .complete
            stateMachineProcess()
            return
        }
        
        startOtaCommandTimer()
        let otaCommand = OtaCommandData(command: .abort, lParam: UInt32(fwImageSize))
        otaDevice.writeValue(to: .controlPointCharacteristic, value: otaCommand.value) { (data, error) in
            guard error == nil else {
                OtaManager.shared.dumpOtaStatus()
                print("error: OtaUpgrader, otaAbort, failed to write Abort command, error:\(String(describing: error))")
                self.completeError = OtaError(state: self.state,
                                              code: OtaErrorCode.ERROR_CHARACTERISTIC_WRITE_VALUE,
                                              desc: "failed to write Abort command")
                OtaNotificationData(otaError: self.completeError!).post()
                self.state = .complete
                self.stateMachineProcess()
                return
            }
        }
    }
    
    private func otaAbortResponse(data: Data?, error: Error?) {
        stopOtaCommandTimer()
        if let error = error {
            print("error: OtaUpgrader, otaAbortResponse, error:\(error)")
            completeError = completeError ?? OtaError(state: state, code: OtaErrorCode.FAILED, desc: "failed to send abort command")
        } else {
            let status = OtaCommandStatus.parse(from: data)
            print("OtaUpgrader, otaAbortResponse, status:\(status.description())")
            if status == .success {
                completeError = completeError ?? OtaError(state: state, code: OtaErrorCode.ERROR_OTA_ABORTED, desc: "fw OTA has been aborted")
            } else {
                completeError = completeError ?? OtaError(state: state, code: OtaErrorCode.INVALID_RESPONSE_VALUE, desc: "otaAbort response status: \(status.description())")
            }
        }
        OtaNotificationData(otaState: state, otaError: nil).post()
        state = .complete
        stateMachineProcess()
    }
    
    private func otaCompleted() {
        stopOtaCommandTimer()
        OtaManager.shared.dumpOtaStatus()
        if completeError == nil {
            print("OtaUpgrader, otaCompleted with success ")
        } else {
            print("OtaUpgrader, ota failed, completed with error:\(String(describing: completeError)) ")
        }
        OtaNotificationData(otaState: .complete, otaError: completeError).post()
    }
}

extension OtaUpgrader {
    public enum OtaState: Int {
        case idle = 0
        case connect = 1
        case otaServiceDiscover = 2
        case readAppInfo = 3
        case enableNotification = 4
        case prepareForDownload = 5
        case startDownload = 6
        case dataTransfer = 7
        case verify = 8
        case abort = 9
        case complete = 10
        
        public var description: String {
            switch self {
            case .idle:
                return "idle"
            case .connect:
                return "connect"
            case .otaServiceDiscover:
                return "otaServiceDiscover"
            case .readAppInfo:
                return "readAppInfo"
            case .enableNotification:
                return "enableNotification"
            case .prepareForDownload:
                return "prepareForDownload"
            case .startDownload:
                return "startDownload"
            case .dataTransfer:
                return "dataTransfer"
            case .verify:
                return "verify"
            case .abort:
                return "abort"
            case .complete:
                return "complete"
            }
        }
    }
    
    private enum OtaCommand: Int {
        case prepareDownload = 1
        case startDownload = 2
        case verify = 3
        case finish = 4
        case getStatus = 5      // not currently used
        case clearStatus = 6    // not currently used
        case abort = 7
    }
    
    private struct OtaCommandData {
        // dataSize: 4 bytes; command: 1 byte; parameters: max 4 bytes
        private var bytes: [UInt8]
        var value: Data {
            return Data(bytes: bytes)
        }
        var count: Int {
            return bytes.count
        }
        
        init(command: OtaCommand) {
            let dataSize = 1
            bytes = [UInt8](repeating: 0, count: dataSize)
            bytes[0] = UInt8(command.rawValue)
        }
        
        init(command: OtaCommand, sParam: UInt16) {
            let dataSize = 3
            bytes = [UInt8](repeating: 0, count: dataSize)
            bytes[0] = UInt8(command.rawValue)
            bytes[1] = UInt8(sParam & 0xFF)
            bytes[2] = UInt8((sParam >> 8) & 0xFF)
        }
        
        init(command: OtaCommand, lParam: UInt32) {
            let dataSize = 5
            bytes = [UInt8](repeating: 0, count: dataSize)
            bytes[0] = UInt8(command.rawValue)
            bytes[1] = UInt8(lParam & 0xFF)
            bytes[2] = UInt8((lParam >> 8) & 0xFF)
            bytes[3] = UInt8((lParam >> 16) & 0xFF)
            bytes[4] = UInt8((lParam >> 24) & 0xFF)
        }
    }
    
    // OTA Command Response status.
    private enum OtaCommandStatus: UInt8 {
        case success = 0
        case unsupported = 1
        case illegal = 2
        case verificationFailed = 3
        case invalidImage = 4
        case invalidImageSize = 5
        case moreData = 6
        case invalidAppId = 7
        case invalidVersion = 8
        case continueStatus = 9
        case sendCommandFailed = 10
        case invalidParameters = 11
        case timeout = 12
        case commandResponseError = 13
        
        static func parse(from data: Data?) -> OtaCommandStatus {
            var status: OtaCommandStatus = .unsupported
            if let respData = data, respData.count > 0 {
                status = OtaCommandStatus.init(rawValue: UInt8(respData[0])) ?? .unsupported
            }
            return status
        }
        
        func description() -> String {
            switch self {
            case .success:
                return "success"
            case .unsupported:
                return "unsupported command"
            case .illegal:
                return "illegal state"
            case .verificationFailed:
                return "image varification failed"
            case .invalidAppId:
                return "invalid App Id"
            case .invalidImage:
                return "invalid image"
            case .invalidImageSize:
                return "invalid image size"
            case .invalidVersion:
                return "invalid version"
            case .moreData:
                return "more data"
            case .continueStatus:
                return "continue"
            case .sendCommandFailed:
                return "failed to write command or data"
            case .invalidParameters:
                return "invalid parameters or invalid objects"
            case .timeout:
                return "timeout"
            case .commandResponseError:
                return "commandResponseError"
            }
        }
    }
}

extension OtaUpgrader {
    private func startOtaCommandTimer() {
        stopOtaCommandTimer()
        
        var interval: TimeInterval = 10
        if state == .connect {
            interval += TimeInterval(exactly: MeshConstants.MESH_DEFAULT_SCAN_DURATION) ?? 30.0
        } else if state == .otaServiceDiscover || state == .verify {
            interval = 30
        }
        
        if #available(iOS 10.0, *) {
            otaCommandTimer = Timer.scheduledTimer(withTimeInterval: interval, repeats: false, block: { (Timer) in
                self.onOtaCommandTimeout()
            })
        } else {
            otaCommandTimer = Timer.scheduledTimer(timeInterval: interval, target: self,
                                                selector: #selector(self.onOtaCommandTimeout),
                                                userInfo: nil, repeats: false)
        }
    }
    
    private func stopOtaCommandTimer() {
        otaCommandTimer?.invalidate()
        otaCommandTimer = nil
    }
    
    @objc private func onOtaCommandTimeout() {
        if state == .readAppInfo {
            // AppInfo read are not supported on some devices, so bypass the error if it happen.
            otaReadAppInfoResponse(data: nil, error: nil)
            return
        } else if state == .idle || state == .connect || state == .otaServiceDiscover || state == .abort {
            state = .complete
        } else {
            if state == .verify {
                self.isGetComponentInfoRunning = false
            }
            state = .abort
        }
        if completeError == nil {
            var error_msg = "execute ota command or write ota data timeout error"
            if !isOtaUpgradePrepareReady {
                error_msg = "OTA service discovering timeout error"
            }
            completeError = OtaError(state: state, code: OtaErrorCode.TIMEOUT, desc: error_msg)
        }
        OtaNotificationData.init(otaError: completeError!).post()
        stateMachineProcess()
    }
}

extension OtaUpgrader {
    /* The max MTU size is 158 on iOS version < 10, and is 185 when iOS version >= 10. */
    static func getMaxDataTransferSize(deviceType: OtaDeviceType) -> Int {
        let mtuSize = PlatformManager.SYSTEM_MTU_SIZE
        if deviceType == .homeKit {
            return 255  // Max 255 without any error, 2 data packets with 1 ack response.
        } else if deviceType == .mesh {
            return (mtuSize - 3 - 17)   // 3 link layer header bytes, exter 17 Mesh encryption bytes
        }
        return (mtuSize - 3)    // 3 link layer header bytes
    }
    
    static let CRC32_INIT_VALUE: UInt32 = 0xFFFFFFFF
    static let crc32Table: [UInt32] = [
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
        0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
        0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
        0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
        0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
        0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
        0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
        0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
        0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
        0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
        0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
        0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
        0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
        0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
        0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
        0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
        0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
        0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
        0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
        0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
        0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
        0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
        0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
        0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
        0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
        0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
        0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
        0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
        0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
        0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
        0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
    ]
    
    /*
     * Help function to calculate CRC32 checksum for specific data.
     *
     * @param crc32     CRC32 value for calculating.
     *                  The initalize CRC value must be OtaUpgrader.CRC32_INIT_VALUE.
     * @param data      Data required to calculate CRC32 checksum.
     *
     * @return          Calculated CRC32 checksum value.
     *
     * Note, after the latest data has been calculated, the final CRC32 checksum value must be calculuated as below as last step:
     *      crc32 ^= OtaUpgrader.CRC32_INIT_VALUE
     */
    static func calculateCrc32(crc32: UInt32, data: Data) -> UInt32 {
        var newCrc = crc32
        var index: Int
        
        for n: UInt8 in data {
            index = Int((newCrc ^ UInt32(n)) & 0xFF)
            newCrc = OtaUpgrader.crc32Table[index] ^ (newCrc >> 8)
        }
        
        return newCrc
    }
}
