//
//  MeshOtaDfuViewController.swift
//  MeshApp
//
//  Created by Dudley Du on 2019/4/2.
//  Copyright © 2019 Cypress Semiconductor. All rights reserved.
//

import UIKit
import MeshFramework

class MeshOtaDfuViewController: UIViewController {
    @IBOutlet weak var dfuNavigationBar: UINavigationBar!
    @IBOutlet weak var dfuNavigationItem: UINavigationItem!
    @IBOutlet weak var dfuNavigationLeftButtonItem: UIBarButtonItem!
    @IBOutlet weak var dfuNavigationRightButtonItem: UIBarButtonItem!
    
    @IBOutlet weak var deviceNameView: UIView!
    @IBOutlet weak var deviceNameLabel: UILabel!
    @IBOutlet weak var deviceTypeLable: UILabel!
    @IBOutlet weak var activityIndicator: UIActivityIndicatorView!
    
    @IBOutlet weak var meshDfuContentView: UIView!
    @IBOutlet weak var dfuTypeLabel: UILabel!
    @IBOutlet weak var dfuTypeDropDownButton: CustomDropDownButton!
    @IBOutlet weak var versionLabel: UILabel!
    @IBOutlet weak var dfuFwImagesDropDownButton: CustomDropDownButton!
    @IBOutlet weak var dfuMetadataImagesDropDownButton: CustomDropDownButton!
    
    @IBOutlet weak var buttonsTopView: UIView!
    @IBOutlet weak var buttunsLeftSubView: UIView!
    @IBOutlet weak var buttonsRightSubView: UIView!
    @IBOutlet weak var getDfuStatusButton: CustomLayoutButton!
    @IBOutlet weak var applyDfuButton: CustomLayoutButton!
    @IBOutlet weak var startUpgradeButton: CustomLayoutButton!
    @IBOutlet weak var stopUpgradeButton: CustomLayoutButton!
    
    @IBOutlet weak var progressView: UIProgressView!
    @IBOutlet weak var upgradeLogLabel: UILabel!
    @IBOutlet weak var upgradePercentageLabel: UILabel!
    @IBOutlet weak var upgradeLogTextView: UITextView!
    
    private var otaBasicDate = Date(timeIntervalSinceNow: 0)
    
    var deviceName: String?
    var groupName: String?  // When groupName is not nil, it comes from CmponentViewControl; if groupName is nil, it comes from FirmwareUpgradeViewController.
    
    var otaDevice: OtaDeviceProtocol?
    var otaFwImageNames: [String] = []
    var otaMetadataImageNames: [String] = []
    var selectedFwImageName: String?
    var selectedMetadataImageName: String?
    
    var isPreparingForOta: Bool = false
    var otaUpdatedStarted: Bool = false
    var lastTransferredPercentage: Int = -1  // indicates invalid value, will be udpated.
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        print("MeshOtaDfuViewController, viewDidLoad")

        // Do any additional setup after loading the view.c
        otaDevice = OtaManager.shared.activeOtaDevice
        notificationInit()
        viewInit()
    }
    
    override func viewDidDisappear(_ animated: Bool) {
        NotificationCenter.default.removeObserver(self)
        OtaManager.shared.resetOtaUpgradeStatus()
        super.viewDidDisappear(animated)
    }
    
    func viewInit() {
        dfuNavigationItem.title = "Mesh DFU"
        versionLabel.text = "Not Avaiable"
        upgradeLogTextView.text = ""
        log("OTA Upgrade view loaded")
        log("OTA device type: \(otaDevice?.getDeviceType() ?? OtaDeviceType.mesh)")
        log("OTA device name: \"\(otaDevice?.getDeviceName() ?? "Not Avaiable")\"")
        
        dfuNavigationItem.rightBarButtonItem = nil  // not used currently.
        upgradeLogTextView.layer.borderWidth = 1
        upgradeLogTextView.layer.borderColor = UIColor.gray.cgColor
        upgradeLogTextView.isEditable = false
        upgradeLogTextView.isSelectable = false
        upgradeLogTextView.layoutManager.allowsNonContiguousLayout = false
        
        otaUpdatedStarted = false
        lastTransferredPercentage = -1  // indicates invalid value, will be udpated.
        otaProgressUpdated(percentage: 0.0)
        
        dfuTypeDropDownButton.dropDownItems = MeshDfuType.DFU_TYPE_TEXT_LIST
        dfuTypeDropDownButton.setSelection(select: MeshDfuType.APP_DFU_TO_DEVICE)
        
        guard let otaDevice = self.otaDevice else {
            print("error: MeshOtaDfuViewController, viewInit, invalid otaDevice instance nil")
            log("error: invalid nil OTA device object")
            DispatchQueue.main.async {
                UtilityManager.showAlertDialogue(parentVC: self, message: "Invalid nil OTA device object.", title: "Error")
            }
            return
        }
        deviceNameLabel.text = otaDevice.getDeviceName()
        deviceTypeLable.text = OtaManager.getOtaDeviceTypeString(by: otaDevice.getDeviceType())
        if otaDevice.getDeviceType() != .mesh {
            dfuTypeDropDownButton.isEnabled = false
        }
        
        applyDfuButton.setTitleColor(UIColor.gray, for: .disabled)
        startUpgradeButton.setTitleColor(UIColor.gray, for: .disabled)
        stopUpgradeButton.setTitleColor(UIColor.gray, for: .disabled)
        stopAnimating()
        
        DispatchQueue.main.async {
            // read and update firmware image list.
            self.firmwareImagesInit()
            //self.dfuFwImagesDropDownButton.dropDownItems = self.otaFwImageNames
            //self.dfuMetadataImagesDropDownButton.dropDownItems = self.otaMetadataImageNames
            self.dfuMetadataImagesDropDownButton.dropDownItems = ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"]
            self.dfuFwImagesDropDownButton.dropDownItems = ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"]
            
            // [Dudley] test purpose.
            // Try to read and show the firmware version if supported before starting the OTA upgrade process.
            // Or remove the return; line to let the App always run the OTA process by click the Firmware Upgrade button.
            // return;
            if let otaDevice = self.otaDevice, otaDevice.getDeviceType() != .mesh, MeshFrameworkManager.shared.isMeshNetworkConnected() {
                MeshFrameworkManager.shared.disconnectMeshNetwork(completion: { (isConnected: Bool, connId: Int, addr: Int, isOverGatt: Bool, error: Int) in
                    self.otaUpgradePrepare()
                })
            } else {
                self.otaUpgradePrepare()
            }
        }
    }
    
    func notificationInit() {
        /* [Dudley]: When do firmware OTA, the mesh notification should be suppressed to avoid any confusion.
         NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
         name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_NODE_CONNECTION_STATUS_CHANGED), object: nil)
         NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
         name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_NETWORK_LINK_STATUS_CHANGED), object: nil)
         NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
         name: Notification.Name(rawValue: MeshNotificationConstants.MESH_NETWORK_DATABASE_CHANGED), object: nil)
         */
        
        NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
                                               name: Notification.Name(rawValue: OtaConstants.Notification.OTA_STATUS_UPDATED), object: nil)
    }
    
    @objc func notificationHandler(_ notification: Notification) {
        guard let userInfo = notification.userInfo else {
            return
        }
        switch notification.name {
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_NODE_CONNECTION_STATUS_CHANGED):
            if let nodeConnectionStatus = MeshNotificationConstants.getNodeConnectionStatus(userInfo: userInfo) {
                self.showToast(message: "Device \"\(nodeConnectionStatus.componentName)\" \((nodeConnectionStatus.status == MeshConstants.MESH_CLIENT_NODE_CONNECTED) ? "has connected." : "is unreachable").")
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_NETWORK_LINK_STATUS_CHANGED):
            if let linkStatus = MeshNotificationConstants.getLinkStatus(userInfo: userInfo) {
                self.showToast(message: "Mesh network has \((linkStatus.isConnected) ? "connected" : "disconnected").")
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_NETWORK_DATABASE_CHANGED):
            if let networkName = MeshNotificationConstants.getNetworkName(userInfo: userInfo) {
                self.showToast(message: "Database of mesh network \(networkName) has changed.")
            }
        case Notification.Name(rawValue: OtaConstants.Notification.OTA_STATUS_UPDATED):
            if let otaStatus = OtaConstants.Notification.getOtaNotificationData(userInfo: userInfo) {
                let otaState = OtaUpgrader.OtaState(rawValue: otaStatus.otaState) ?? OtaUpgrader.OtaState.idle
                if otaStatus.errorCode == OtaErrorCode.SUCCESS {
                    if otaStatus.otaState == OtaUpgrader.OtaState.idle.rawValue {
                        log("OTA state: \(otaStatus.description).")
                    } else if otaStatus.otaState == OtaUpgrader.OtaState.readAppInfo.rawValue {
                        // try to get and show the read firmware version from remote device.
                        let appInfo = String(otaStatus.description.trimmingCharacters(in: CharacterSet.whitespaces))
                        log("OTA state: \(otaState.description) finished success. \(appInfo)")
                        var version = ""
                        if let otaDevice = self.otaDevice, otaDevice.getDeviceType() == .mesh, appInfo.hasPrefix("CID:") {
                            let componentInfoValue = MeshComponentInfo(componentInfo: appInfo)
                            version = componentInfoValue.VID
                        } else {
                            let appVersion = String(appInfo.split(separator: " ").last ?? "")
                            let characterSet = CharacterSet(charactersIn: "0123456789.")
                            version = appVersion.trimmingCharacters(in: characterSet)
                        }
                        if !version.isEmpty {
                            versionLabel.text = version
                        }
                    } else if otaStatus.otaState == OtaUpgrader.OtaState.dataTransfer.rawValue {
                        if otaStatus.transferredImageSize == 0 {
                            log("OTA state: \(otaState.description) started.")
                        }
                        // Update and log firmware image download percentage value.
                        otaProgressUpdated(percentage: Float(otaStatus.transferredImageSize) / Float(otaStatus.fwImageSize))
                    } else if otaStatus.otaState == OtaUpgrader.OtaState.complete.rawValue {
                        otaUpdatedStarted = false
                        // OTA upgrade process finished, navigate to previous view controller if success.
                        self.stopAnimating()
                        if !self.isPreparingForOta {
                            self.log("done: OTA upgrade completed success.\n")
                            UtilityManager.showAlertDialogue(parentVC: self,
                                                             message: "Congratulation! OTA process has finshed successfully.",
                                                             title: "Success", completion: nil,
                                                             action: UIAlertAction(title: "OK", style: .default,
                                                                                   handler: { (action) in
                                                                                    self.onDfuNavigationLeftButtonItemClick(self.dfuNavigationLeftButtonItem)
                                                             }))
                        } else {
                            self.log("done: prepare for OTA upgrade is ready.\nPlease select a firmware image and click the Firmware Upgrade button to start OTA.\n")
                        }
                        self.isPreparingForOta = false
                    } else {
                        // Log normal OTA upgrade successed step.
                        log("OTA state: \(otaState.description) finished success.")
                    }
                } else {
                    if otaStatus.otaState == OtaUpgrader.OtaState.complete.rawValue {
                        otaUpdatedStarted = false
                        // OTA upgrade process finished
                        self.log("done: OTA upgrade stopped with error. Error Code: \(otaStatus.errorCode), \(otaStatus.description)\n")
                        self.stopAnimating()
                        if !self.isPreparingForOta {
                            UtilityManager.showAlertDialogue(parentVC: self,
                                                             message: "Oops! OTA process stopped with some error, please reset device and retry again later.")
                        } else {
                            if otaStatus.errorCode == OtaErrorCode.ERROR_DEVICE_OTA_NOT_SUPPORTED {
                                UtilityManager.showAlertDialogue(parentVC: self,
                                                                 message: "Oops! Target device doesn't support Cypress OTA function, please select the device that support Cypress OTA function and try again.",
                                                                 title: "Error", completion: nil,
                                                                 action: UIAlertAction(title: "OK", style: .default,
                                                                                       handler: { (action) in
                                                                                        self.onDfuNavigationLeftButtonItemClick(self.dfuNavigationLeftButtonItem)
                                                                 }))
                            } else {
                                self.log("Please select the firmare image, then click the \"Firmware Upgrade\" button to try again.\n\n")
                            }
                        }
                        self.isPreparingForOta = false
                    } else {
                        // Log normal OTA upgrade failed step.
                        log("error: OTA state: \(otaState.description) failed. Error Code:\(otaStatus.errorCode), message:\(otaStatus.description)")
                    }
                }
            }
        default:
            break
        }
    }
    
    //
    // This function will try to connect to the OTA device, then try to discover OTA services,
    // try and read the AppInfo and update to UI if possible.
    // Also, to call this function alonely can help to verify the feature of connecting to different OTA devices.
    //
    func doOtaUpgradePrepare() {
        guard let otaDevice = self.otaDevice else {
            print("error: MeshOtaDfuViewController, otaUpgradePrepare, invalid OTA device instance")
            log("error: invalid nil OTA device object")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Invalid nil OTA device object.")
            return
        }
        
        isPreparingForOta = true
        startAnimating()
        let error = otaDevice.prepareOta()
        guard error == OtaErrorCode.SUCCESS else {
            stopAnimating()
            if error == OtaErrorCode.BUSYING {
                return
            }
            print("error: MeshOtaDfuViewController, otaUpgradePrepare, failed to prepare for OTA")
            self.log("error: failed to prepare for OTA, error:\(error)")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to prepare for OTA. Error Code: \(error).", title: "Error")
            return
        }
    }
    func otaUpgradePrepare() {
        DispatchQueue.main.async {
            self.doOtaUpgradePrepare()  // because self.otaDevice instance is from main thread, so make suer it running in main thread.
        }
    }
    
    func firmwareImagesInit() {
        let meshFwPath = "mesh/fwImages"
        let fwImagesPath = NSHomeDirectory() + "/Documents/" + meshFwPath
        let defaultDocumentsPath = NSHomeDirectory() + "/Documents"
        
        otaFwImageNames.removeAll()
        otaMetadataImageNames.removeAll()
        let foundInFwImages = addFirmwareImageNames(atPath: fwImagesPath)
        let foundInDocuments = addFirmwareImageNames(atPath: defaultDocumentsPath)
        if !foundInFwImages, !foundInDocuments {
            print("error: MeshOtaDfuViewController, firmwareImagesInit, no valid firmware images found")
            UtilityManager.showAlertDialogue(parentVC: self, message: "No valid firmware images found under both App's Documents directory and \"Documents/\(meshFwPath)\" directory. Please copy valid firmware images into your device, then try again later.", title: "Error")
        }
    }
    
    func addFirmwareImageNames(atPath: String) -> Bool {
        var isDirectory = ObjCBool(false)
        let exists = FileManager.default.fileExists(atPath: atPath, isDirectory: &isDirectory)
        if !exists || !isDirectory.boolValue {
            print("error: MeshOtaDfuViewController, addFirmwareImageNames, \(atPath) not exsiting")
            return false
        }
        
        if let files = try? FileManager.default.contentsOfDirectory(atPath: atPath) {
            for fileName in files {
                if fileName.hasSuffix(".bin") {
                    otaFwImageNames.append(fileName)
                    print("MeshOtaDfuViewController, addFirmwareImageNames, found image: \(fileName)")
                } else if fileName.hasPrefix("image_info") {
                    otaMetadataImageNames.append(fileName)
                    print("MeshOtaDfuViewController, addFirmwareImageNames, found metadata image: \(fileName)")
                }
            }
        }
        
        if otaFwImageNames.isEmpty, otaMetadataImageNames.isEmpty {
            return false
        }
        return true
    }
    
    func otaProgressUpdated(percentage: Float) {
        let pct: Float = (percentage > 1.0) ? 1.0 : ((percentage < 0.0) ? 0.0 : percentage)
        let latestPercentage = Int(pct * 100)
        upgradePercentageLabel.text = String(format: "%d", latestPercentage) + "%"
        progressView.progress = percentage
        
        if otaUpdatedStarted, lastTransferredPercentage != latestPercentage {
            log("transferred size: \(latestPercentage)%%")
            lastTransferredPercentage = latestPercentage
        }
    }
    
    func log(_ message: String) {
        let seconds = Date().timeIntervalSince(otaBasicDate)
        let msg = String(format: "[%.3f] \(message)\n", seconds)
        upgradeLogTextView.text += msg
        let bottom = NSRange(location: upgradeLogTextView.text.count, length: 1)
        upgradeLogTextView.scrollRangeToVisible(bottom)
    }
    
    func startAnimating() {
        applyDfuButton.isEnabled = false
        startUpgradeButton.isEnabled = false
        stopUpgradeButton.isEnabled = false
        activityIndicator.startAnimating()
        activityIndicator.isHidden = false
    }
    
    func stopAnimating() {
        activityIndicator.stopAnimating()
        activityIndicator.isHidden = true
        applyDfuButton.isEnabled = true
        startUpgradeButton.isEnabled = true
        stopUpgradeButton.isEnabled = true
    }
    

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */
    
    @IBAction func onDfuNavigationLeftButtonItemClick(_ sender: UIBarButtonItem) {
        print("MeshOtaDfuViewController, onDfuNavigationLeftButtonItemClick")
        OtaManager.shared.resetOtaUpgradeStatus()
        if let otaDevice = self.otaDevice, otaDevice.getDeviceType() == .mesh, let groupName = self.groupName {
            print("MeshOtaDfuViewController, navigate back to ComponentViewController page)")
            UserSettings.shared.currentActiveGroupName = groupName
            UserSettings.shared.currentActiveComponentName = otaDevice.getDeviceName()
            UtilityManager.navigateToViewController(targetClass: ComponentViewController.self)
        } else {
            print("MeshOtaDfuViewController, navigate to FirmwareUpgradeViewController page)")
            if let otaDevice = self.otaDevice, otaDevice.getDeviceType() != .mesh {
                otaDevice.disconnect()
            }
            UtilityManager.navigateToViewController(targetClass: FirmwareUpgradeViewController.self)
        }
        self.dismiss(animated: true, completion: nil)
    }
    
    @IBAction func onDfuNavigationRightButtonItemClick(_ sender: UIBarButtonItem) {
    }
    
    @IBAction func onDfuTypeDropDownButtonClick(_ sender: CustomDropDownButton) {
        dfuTypeDropDownButton.showDropList(width: 220, parent: self) {
            print("\(self.dfuTypeDropDownButton.selectedIndex), \(self.dfuTypeDropDownButton.selectedString)")
        }
    }
    
    @IBAction func onDfuFwImagesDropDownButtonClick(_ sender: CustomDropDownButton) {
        let width = Int(meshDfuContentView.bounds.size.width) - 16
        dfuFwImagesDropDownButton.showDropList(width: width, parent: self) {
            print("\(self.dfuFwImagesDropDownButton.selectedIndex), \(self.dfuFwImagesDropDownButton.selectedString)")
        }
    }
    
    @IBAction func onDfuMetadataImagesDropDownButtonClick(_ sender: CustomDropDownButton) {
        let width = Int(meshDfuContentView.bounds.size.width) - 16
        dfuMetadataImagesDropDownButton.showDropList(width: width, parent: self) {
            print("\(self.dfuMetadataImagesDropDownButton.selectedIndex), \(self.dfuMetadataImagesDropDownButton.selectedString)")
        }
    }
    
    @IBAction func onGetDfuStatusButtonClick(_ sender: CustomLayoutButton) {
    }
    
    @IBAction func onApplyDfuButtonClick(_ sender: CustomLayoutButton) {
    }
    
    @IBAction func onStartUpgradeButtonClick(_ sender: CustomLayoutButton) {
    }
    
    @IBAction func onStopUpgradeButtonClick(_ sender: CustomLayoutButton) {
    }
}
