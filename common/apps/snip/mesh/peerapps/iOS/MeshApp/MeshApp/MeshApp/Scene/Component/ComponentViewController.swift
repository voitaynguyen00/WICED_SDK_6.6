/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * Component device detail view controller implementation.
 */

import UIKit
import MeshFramework

enum ComponentPopoverChoices: String {
    case identify = "Identify"
    case rename = "Rename Device"
    case delete = "Delete Device"
    case move = "Move to Other Group"
    case add = "Add to Other Group"
    case remove = "Remove from this Group"
    case ota = "Firmware OTA"
    
    static var allValues = [ComponentPopoverChoices.identify.rawValue,
                            ComponentPopoverChoices.rename.rawValue,
                            ComponentPopoverChoices.delete.rawValue,
                            ComponentPopoverChoices.add.rawValue,
                            ComponentPopoverChoices.move.rawValue,
                            ComponentPopoverChoices.remove.rawValue,
                            ComponentPopoverChoices.ota.rawValue]
}

class ComponentViewController: UIViewController {
    @IBOutlet weak var backBarButtonItem: UIBarButtonItem!
    @IBOutlet weak var settingBarButtonItem: UIBarButtonItem!
    @IBOutlet weak var contentView: UIView!
    
    var groupName: String?
    var deviceName: String?
    var componentType: Int = MeshConstants.MESH_COMPONENT_UNKNOWN
    
    private var operationType: ComponentPopoverChoices?
    private var componentOperationTimer: Timer?
    private let indicatorView = CustomIndicatorView()
    private var popoverSelectedGroupName: String?
    
    private var mConponentControlVC: UIViewController?
    lazy var conponentControlVC: UIViewController = {
        if let vc = self.mConponentControlVC {
            return vc
        }
        
        if componentType >= MeshConstants.MESH_COMPONENT_SENSOR_SERVER {
            guard let controlVC = self.storyboard?.instantiateViewController(withIdentifier: MeshAppStoryBoardIdentifires.SENSOR) as? SensorViewController else {
                print("error: ComponentViewController, failed to load \(MeshAppStoryBoardIdentifires.SENSOR) ViewController from Main storyboard")
                return UIViewController()
            }
            return controlVC as UIViewController
        }
        
        guard let controlVC = self.storyboard?.instantiateViewController(withIdentifier: MeshAppStoryBoardIdentifires.GROUP_DETAILS_CONTROLS) as? GroupDetailsControlsViewController else {
            print("error: ComponentViewController, failed to load \(MeshAppStoryBoardIdentifires.GROUP_DETAILS_CONTROLS) ViewController from Main storyboard")
            return UIViewController()
        }
        return controlVC as UIViewController
    }()
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        notificationInit()
        contentViewInit()
    }
    
    override func viewDidDisappear(_ animated: Bool) {
        NotificationCenter.default.removeObserver(self)
        super.viewDidDisappear(animated)
    }
    
    func notificationInit() {
        NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_NODE_CONNECTION_STATUS_CHANGED), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_NETWORK_LINK_STATUS_CHANGED), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_NETWORK_DATABASE_CHANGED), object: nil)
        
        NotificationCenter.default.addObserver(self, selector: #selector(notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_NETWORK_DATABASE_CHANGED), object: nil)
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
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_NETWORK_DATABASE_CHANGED):
            guard let dbChangedNetworkName = MeshNotificationConstants.getNetworkName(userInfo: userInfo),
                dbChangedNetworkName == UserSettings.shared.currentActiveMeshNetworkName else {
                    print("error, ComponentViewController, onNetworkDatabaseChanged, DB change event network name not match")
                    return
            }
            
            componentOperationTimer?.invalidate()
            componentOperationTimer = nil
            
            if let opType = operationType {
                switch opType {
                case .identify:
                    onDeviceIdentify()
                case .delete:
                    onComponentDeviceDelete()
                case .move:
                    onMovedToGroupHandler(.none, nil)
                case .rename:
                    break
                case .add:
                    onAddToGroupHandler(.none, nil)
                case .remove:
                    break
                case .ota:
                    break
                }
            }
        default:
            break
        }
    }
    
    func contentViewInit() {
        self.groupName = UserSettings.shared.currentActiveGroupName
        self.deviceName = UserSettings.shared.currentActiveComponentName ?? MeshConstantText.UNKNOWN_DEVICE_NAME
        if let deviceName = self.deviceName {
            self.componentType = MeshFrameworkManager.shared.getMeshComponentType(componentName: deviceName)
        }
        print("ComponentViewController, contentViewInit, devceName:\(String(describing: self.deviceName)), componentType:\(self.componentType)")
        if componentType >= MeshConstants.MESH_COMPONENT_SENSOR_SERVER {
            let vc = self.conponentControlVC
            if let controlVC = vc as? SensorViewController {
                controlVC.groupName = self.groupName
                controlVC.deviceName = self.deviceName
                controlVC.componentType = componentType
            }
        } else {
            let vc = self.conponentControlVC
            if let controlVC = vc as? GroupDetailsControlsViewController {
                controlVC.isDeviceControl = true
                controlVC.groupName = self.groupName
                controlVC.deviceName = self.deviceName
                controlVC.componentType = componentType
            }
        }
        self.addChild(self.conponentControlVC)
        self.conponentControlVC.didMove(toParent: self)
        self.conponentControlVC.view.frame = self.contentView.bounds
        self.contentView.addSubview(self.conponentControlVC.view)
    }
    
    // MARK: - Navigation
    
    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
        if let identifier = segue.identifier {
            print("ComponentViewController, segue.identifier=\(identifier)")
            switch identifier {
            case MeshAppStoryBoardIdentifires.SEGUE_COMPONENT_BACK_TO_GROUP_DETAILS:
                if let groupDetailVC = segue.destination as? GroupDetailsViewController {
                    groupDetailVC.currentSegmentedSelectedIndex = MeshAppConstants.GROUPT_DETAIL_SEGMENTED_ALL_DEVICE
                }
            default:
                break
            }
        }
    }
    
    @IBAction func onSettingBarButtonItemClick(_ sender: UIBarButtonItem) {
        print("ComponentViewController, onSettingBarButtonItemClick")
        guard let currentGroupName = self.groupName else {
            print("ComponentViewController, onSettingBarButtonItemClick, invalid group name nil")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Show Setting button list table failed, invalid group name.")
            return
        }
        var choices = ComponentPopoverChoices.allValues
        if currentGroupName == MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME {
            choices.removeAll(where: {$0 == ComponentPopoverChoices.move.rawValue})
            choices.removeAll(where: {$0 == ComponentPopoverChoices.remove.rawValue})
        }
        let controller = PopoverChoiceTableViewController(choices: choices) { (index: Int, selection: String) in
            print("ComponentViewController, onSettingBarButtonItemClick, index=\(index), selection=\(selection)")
            guard let choice = ComponentPopoverChoices.init(rawValue: selection) else { return }
            
            switch choice {
            case .identify:
                self.onDeviceIdentify()
            case .rename:
                self.onComponentDeviceRename()
            case .delete:
                self.onComponentDeviceDelete()
            case .move:
                self.onComponentDeviceMoveToGroup()
            case .add:
                self.onComponentDeviceAddToGroup()
            case .remove:
                self.onComponentRemoveFromCurrentGroup()
            case .ota:
                self.onFirmwareOta()
            }
        }
        controller.preferredContentSize = CGSize(width: 220, height: controller.getPreferredPopoverViewSize())
        controller.showPopoverPresentation(parent: self, sourceView: sender.value(forKey: "view") as? UIView)
    }
    
    func onDeviceIdentify() {
        print("ComponentViewController, onDeviceIdentify")
        guard let deviceName = self.deviceName else {
            print("error: ComponentViewController, onDeviceIdentify, invalid deviceName nil")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Invalid device name or nil.")
            return
        }
        
        MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error: Int) in
            guard error == MeshErrorCode.MESH_SUCCESS else {
                print("ComponentViewController, deviceIdentify, device:\(deviceName), failed to connect to mesh network")
                return
            }
            
            let error = MeshFrameworkManager.shared.meshClientIdentify(name: deviceName, duration: 10)
            print("ComponentViewController, deviceIdentify, device:\(deviceName), error:\(error)")
        }
    }
    
    func onComponentDeviceRename() {
        print("ComponentViewController, onComponentDeviceRename, show input new name dialogue")
        let alertController = UIAlertController(title: ComponentPopoverChoices.rename.rawValue, message: nil, preferredStyle: .alert)
        alertController.addTextField { (textField: UITextField) in
            textField.placeholder = "Enter New Device Name"
        }
        alertController.addAction(UIAlertAction(title: "Cancel", style: .default, handler: nil))
        alertController.addAction(UIAlertAction(title: "Confirm", style: .default, handler: { (action: UIAlertAction) -> Void in
            guard let textField = alertController.textFields?.first,
                let newDeviceName = textField.text, newDeviceName.count > 0,
                let oldDeviceName = self.deviceName else {
                UtilityManager.showAlertDialogue(parentVC: self, message: "Empty or Invalid new device name!", title: "Error")
                return
            }
            
            self.operationType = .rename
            self.indicatorView.showAnimating(parentVC: self)
            MeshFrameworkManager.shared.meshClientRename(oldName: oldDeviceName, newName: newDeviceName) { (networkName: String?, error: Int) in
                self.indicatorView.stopAnimating()
                self.operationType = nil
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("error: ComponentViewController, failed to call meshClientRename with new deviceName=\(newDeviceName), error=\(error)")
                    UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to rename \"\(oldDeviceName)\" device. Error Code: \(error)")
                    return
                }
                
                /*
                 * [Dudley]:
                 * Mesh library will always append the mesh unique address string as suffix to avoid device name conflict, the format is " (XXXX)".
                 * Note, the suffix is started to a space, and the XXXX is a for hexdecail value characters.
                 * so, the new new for the mesh device should also be apppended with the mesh unique address string before using it.
                 */
                let newMeshDeviceName = newDeviceName + UtilityManager.getMeshAddressSuffixString(meshComponentName: oldDeviceName)
                self.deviceName = newMeshDeviceName
                if self.componentType >= MeshConstants.MESH_COMPONENT_SENSOR_SERVER,
                    let controlVC = self.conponentControlVC as? SensorViewController {
                    controlVC.deviceName = self.deviceName
                } else if let controlVC = self.conponentControlVC as? GroupDetailsControlsViewController {
                    controlVC.deviceName = self.deviceName
                    controlVC.nameLabel.text = self.deviceName  // update the device name shown in the control view.
                }
                // update device status values in UserSettings.
                if let values = UserSettings.shared.getComponentStatus(componentName: oldDeviceName) {
                    UserSettings.shared.removeComponentStatus(componentName: oldDeviceName)
                    UserSettings.shared.setComponentStatus(componentName: newMeshDeviceName, values: values)
                }
                
                print("ComponentViewController, rename \"\(oldDeviceName)\" device to new name=\"\(newMeshDeviceName)\" success")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Rename device from \"\(oldDeviceName)\" to \"\(newMeshDeviceName)\" sucess", title: "Success")
            }
        }))
        self.present(alertController, animated: true, completion: nil)
    }
    
    func onComponentDeviceDelete() {
        guard let deviceName = self.deviceName else {
            print("error: ComponentViewController, onComponentDeviceDeletedelete, invalid deviceName:\(String(describing: self.deviceName))")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to delete the target device!", title: "Error")
            return
        }
        
        self.operationType = .delete
        self.componentOperationTimer?.invalidate()
        self.componentOperationTimer = Timer.scheduledTimer(timeInterval: 20, target: self, selector: #selector(self.componentOperationTimeoutHandler), userInfo: nil, repeats: false)
        indicatorView.showAnimating(parentVC: self)
        MeshFrameworkManager.shared.meshClientDeleteDevice(deviceName: deviceName) { (_ networkName: String?, _ error: Int) in
            self.componentOperationTimer?.invalidate()
            self.componentOperationTimer = nil
            self.indicatorView.stopAnimating()
            self.operationType = nil
            guard error == MeshErrorCode.MESH_SUCCESS else {
                print("error: ComponentViewController, onComponentDeviceDeletedelete, meshClientDeleteDevice deviceName:\(deviceName) failed, error=\(error)")
                if error == MeshErrorCode.MESH_ERROR_INVALID_STATE {
                    UtilityManager.showAlertDialogue(parentVC: self, message: "Mesh network is busying, please try again to delete device \"\(deviceName)\" a little later.", title: "Warnning")
                } else {
                    UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to delete device \"\(deviceName)\". Error Code: \(error).", title: "Error")
                }
                return
            }
            
            // update device status values in UserSettings.
            UserSettings.shared.removeComponentStatus(componentName: deviceName)
            
            print("ComponentViewController, onComponentDeviceDelete, the mesh device: \(deviceName) has been deleted success, error=\(error)")
            UtilityManager.showAlertDialogue(parentVC: self,
                                             message: "The mesh device has been deleted successfully.",
                                             title: "Success", completion: nil,
                                             action: UIAlertAction(title: "OK", style: .default,
                                                                   handler: { (action) in
                                                                    self.performSegue(withIdentifier: MeshAppStoryBoardIdentifires.SEGUE_COMPONENT_BACK_TO_GROUP_DETAILS, sender: nil)
                                             }))
        }
    }
    
    func onComponentDeviceAddToGroup() {
        print("ComponentViewController, onComponentDeviceAddToGroup, show input new name dialogue")
        if let groupList = getPopoverSelectionItemList(popoverType: .componentMoveToGroup) {
            PopoverViewController.parentViewController = self
            PopoverViewController.popoverCompletion = onAddToGroupHandler
            PopoverViewController.popoverType = .componentAddToGroup
            PopoverViewController.popoverItems = groupList
            
            UtilityManager.navigateToViewController(sender: self, targetVCClass: PopoverViewController.self, modalPresentationStyle: UIModalPresentationStyle.overCurrentContext)
        }
    }
    
    func onAddToGroupHandler(_ btnType: PopoverButtonType, _ selectedItem: String?) {
        guard let deviceName = self.deviceName else {
            print("error: ComponentViewController, onAddToGroupHandler, invalid device name or group name")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Invalid device name or group name.")
            return
        }
        
        // on DB changed callback process.
        if btnType == .none && operationType == .add, let newGroupName = popoverSelectedGroupName {
            componentOperationTimer?.invalidate()
            componentOperationTimer = nil
            indicatorView.stopAnimating()
            operationType = nil
            popoverSelectedGroupName = nil
            guard let componets = MeshFrameworkManager.shared.getMeshGroupComponents(groupName: newGroupName), componets.filter({$0 == deviceName}).count > 0 else {
                print("error: ComponentViewController, onAddToGroupHandler, failed to find device:\(deviceName) in new group:\(newGroupName)")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to add device:\(deviceName) in new group:\(newGroupName)")
                return
            }
            
            // The mesh device has been added to new success.
            print("ComponentViewController, onAddToGroupHandler, add device:\(deviceName) to:\(newGroupName) success")
            UtilityManager.showAlertDialogue(parentVC: self,
                                             message: "The mesh device has been added to new group successfully.",
                                             title: "Success", completion: nil,
                                             action: UIAlertAction(title: "OK", style: .default,
                                                                   handler: { (action) in
                                                                    self.performSegue(withIdentifier: MeshAppStoryBoardIdentifires.SEGUE_COMPONENT_BACK_TO_GROUP_DETAILS, sender: nil)
                                             }))
            return
        }
        
        // Porcess on user selection for delete device.
        componentOperationTimer?.invalidate()
        indicatorView.stopAnimating()
        operationType = nil
        popoverSelectedGroupName = nil
        guard btnType == .confirm, let newGroupName = selectedItem, !newGroupName.isEmpty else {
            if btnType == .confirm {
                print("error: ComponentViewController, onAddToGroupHandler, no add to group name selected")
                UtilityManager.showAlertDialogue(parentVC: self, message: "No or invalid add to group name selected")
            } else {
                print("ComponentViewController, onAddToGroupHandler, cancelled")
            }
            return
        }
        
        indicatorView.showAnimating(parentVC: self)
        MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error) in
            guard error == MeshErrorCode.MESH_SUCCESS else {
                self.indicatorView.stopAnimating()
                print("ComponentViewController, onMovedToGroupSelected, unable to connect to the mesh network")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to connect to mesh network caused by unable to add the device to new group. Please try to sync network systus firstly.")
                return
            }
            
            self.operationType = .add
            self.popoverSelectedGroupName = newGroupName
            self.componentOperationTimer = Timer.scheduledTimer(timeInterval: 20, target: self, selector: #selector(self.componentOperationTimeoutHandler), userInfo: nil, repeats: false)
            MeshFrameworkManager.shared.addMeshComponent(componentName: deviceName, toGroup: newGroupName, completion: { (networkName: String?, error: Int) in
                self.componentOperationTimer?.invalidate()
                self.componentOperationTimer = nil
                self.indicatorView.stopAnimating()
                self.popoverSelectedGroupName = nil
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("error: ComponentViewController, onAddToGroupHandler, add device:\(deviceName) to new group:\(newGroupName) failed, error=\(error)")
                    UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to add the device to selected group. Error Code: \(error)")
                    return
                }
                print("ComponentViewController, onAddToGroupHandler, addMeshComponent command sent success, waiting mesh DB changed event")
            })
        }
    }
    
    func onComponentDeviceMoveToGroup() {
        print("ComponentViewController, onComponentDeviceMoveToGroup, show input new name dialogue")
        if let groupList = getPopoverSelectionItemList(popoverType: .componentMoveToGroup) {
            PopoverViewController.parentViewController = self
            PopoverViewController.popoverCompletion = onMovedToGroupHandler
            PopoverViewController.popoverType = .componentMoveToGroup
            PopoverViewController.popoverItems = groupList
            
            UtilityManager.navigateToViewController(sender: self, targetVCClass: PopoverViewController.self, modalPresentationStyle: UIModalPresentationStyle.overCurrentContext)
        }
    }
    
    func onMovedToGroupHandler(_ btnType: PopoverButtonType, _ selectedItem: String?) {
        guard let deviceName = self.deviceName, let groupName = self.groupName else {
            print("error: ComponentViewController, onMovedToGroupSelected, invalid device name or group name")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Invalid device name or group name.")
            return
        }
        
        // on DB changed callback process.
        if btnType == .none && operationType == .move, let newGroupName = popoverSelectedGroupName {
            componentOperationTimer?.invalidate()
            componentOperationTimer = nil
            indicatorView.stopAnimating()
            operationType = nil
            popoverSelectedGroupName = nil
            guard let groups = MeshFrameworkManager.shared.getMeshComponenetGroupList(componentName: deviceName), groups.count > 0 else {
                print("error: ComponentViewController, onMovedToGroupHandler, failed to get device group list, groups is nil")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to connect to mesh network caused by unable to move the device to new group. Please try to sync network status firstly.")
                return
            }
            
            if groups.filter({$0 == newGroupName}).first != nil &&
                (groups.filter({$0 == groupName}).first == MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME || groups.filter({$0 == groupName}).first == nil) {
                // The mesh device has been moved success.
                print("ComponentViewController, onNetworkDatabaseChanged, move device:\(deviceName) from:\(groupName) to:\(String(describing: popoverSelectedGroupName)) success")
                UtilityManager.showAlertDialogue(parentVC: self,
                                                 message: "The mesh device has been moved to new group successfully.",
                                                 title: "Success", completion: nil,
                                                 action: UIAlertAction(title: "OK", style: .default,
                                                                       handler: { (action) in
                                                                        self.performSegue(withIdentifier: MeshAppStoryBoardIdentifires.SEGUE_COMPONENT_BACK_TO_GROUP_DETAILS, sender: nil)
                                                 }))
            } else {
                print("error: ComponentViewController, onNetworkDatabaseChanged, failed to move device:\(deviceName) from:\(groupName) to:\(String(describing: popoverSelectedGroupName))")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to move mesh device to new group.")
            }
            return
        }
        
        // Porcess on user selection for delete device.
        componentOperationTimer?.invalidate()
        componentOperationTimer = nil
        indicatorView.stopAnimating()
        operationType = nil
        popoverSelectedGroupName = nil
        guard btnType == .confirm, let newGroupName = selectedItem, !newGroupName.isEmpty else {
            if btnType == .confirm {
                print("error: ComponentViewController, onMovedToGroupSelected, no moved to group name selected")
                UtilityManager.showAlertDialogue(parentVC: self, message: "No or invalid moved to group name selected")
            } else {
                print("ComponentViewController, onMovedToGroupSelected, cancelled")
            }
            return
        }
        
        indicatorView.showAnimating(parentVC: self)
        MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error) in
            guard error == MeshErrorCode.MESH_SUCCESS else {
                self.indicatorView.stopAnimating()
                print("ComponentViewController, onMovedToGroupSelected, unable to connect to the mesh network, error=\(error)")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to connect to mesh network caused by unable to move the device to new group. Please try to sync network systus firstly.")
                return
            }

            self.operationType = .move
            self.popoverSelectedGroupName = newGroupName
            self.componentOperationTimer = Timer.scheduledTimer(timeInterval: 20, target: self, selector: #selector(self.componentOperationTimeoutHandler), userInfo: nil, repeats: false)

            // In this demo, the default group will contain all devices, when the device is moved from the default group, it means added to the new group.
            if groupName == MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME {
                if let subscribedGroups = MeshFrameworkManager.shared.getMeshComponenetGroupList(componentName: deviceName) {
                    let nonDefaultGroups = subscribedGroups.filter({$0 != MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME})
                    if nonDefaultGroups.count > 0 {
                        self.componentOperationTimer?.invalidate()
                        self.componentOperationTimer = nil
                        self.indicatorView.stopAnimating()
                        self.popoverSelectedGroupName = nil
                        // The device has been added into other group beside the default group.
                        print("error: ComponentViewController, onMovedToGroupHandler, \(deviceName) has been added in the \(nonDefaultGroups[0]) group")
                        UtilityManager.showAlertDialogue(parentVC: self, message: "Unable to move this device to group \"\(newGroupName)\", it has been added in the \"\(nonDefaultGroups[0])\" group. Please navigate to the \"\(nonDefaultGroups[0])\" group detail page, then move this device to the target group to make sure it's the expected operation.")
                        return
                    } else {
                        // The device only exsiting in the default group, so add it to the target group.
                        MeshFrameworkManager.shared.addMeshComponent(componentName: deviceName, toGroup: newGroupName) { (networkName: String?, error: Int) in
                            
                        }
                    }
                } else {
                    // The device doesn't exist in any group, try to added into new group.
                    MeshFrameworkManager.shared.addMeshComponent(componentName: deviceName, toGroup: newGroupName) { (networkName: String?, error: Int) in
                        
                    }
                }
            } else {
                MeshFrameworkManager.shared.moveMeshComponent(componentName: deviceName, fromGroup: groupName, toGroup: newGroupName) { (networkName: String?, error: Int) in
                    guard error == MeshErrorCode.MESH_SUCCESS else {
                        self.componentOperationTimer?.invalidate()
                        self.componentOperationTimer = nil
                        self.indicatorView.stopAnimating()
                        self.popoverSelectedGroupName = nil
                        print("error: ComponentViewController, moveMeshComponent, \(deviceName) from \(groupName) to \(newGroupName) failed, error=\(error)")
                        UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to move the device to selected group. Error Code: \(error)")
                        return
                    }
                    print("ComponentViewController, moveMeshComponent command sent success, waiting mesh DB changed event")
                }
            }
            guard error == MeshErrorCode.MESH_SUCCESS else {
                self.componentOperationTimer?.invalidate()
                self.componentOperationTimer = nil
                self.indicatorView.stopAnimating()
                self.popoverSelectedGroupName = nil
                print("error: ComponentViewController, moveMeshComponent, \(deviceName) from \(groupName) to \(newGroupName) failed, error=\(error)")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to move the device to selected group. Error Code: \(error)")
                return
            }
            print("ComponentViewController, moveMeshComponent command sent success, waiting mesh DB changed event")
        }
    }
    
    @objc func componentOperationTimeoutHandler() {
        componentOperationTimer?.invalidate()
        componentOperationTimer = nil
        print("error: ComponentViewController, componentOperationTimeoutHandler, operation timeout, operationType=\(String(describing: operationType?.rawValue))")
        if let opType = operationType {
            switch opType {
            case .delete:
                onComponentDeviceDelete()
            case .move:
                onMovedToGroupHandler(.none, nil)
            default:
                break
            }
        }
    }
    
    func getPopoverSelectionItemList(popoverType: PopoverType) -> [String]? {
        guard let deviceName = self.deviceName,
            let networkName = MeshFrameworkManager.shared.getOpenedMeshNetworkName(),
            var groupList = MeshFrameworkManager.shared.getAllMeshNetworkGroups(networkName: networkName) else {
                print("error: ComponentViewController, getPopoverGroupList, Failed to get group list")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to get group list.")
                return nil
        }
        // Do not show the group names that currently subscribed for this device.
        if let subscribedGroups = MeshFrameworkManager.shared.getMeshComponenetGroupList(componentName: deviceName) {
            for group in subscribedGroups {
                groupList.removeAll(where: {$0 == group})
            }
        }
        groupList.removeAll(where: {$0 == MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME})
        guard groupList.count > 0 else {
            print("error: ComponentViewController, getPopoverGroupList, No other group can be added or moved to. Currnetly scribed groups: \(groupList)")
            UtilityManager.showAlertDialogue(parentVC: self, message: "No other group can be selected. Please create a new group firstly.")
            return nil
        }
        
        return groupList
    }
    
    func onComponentRemoveFromCurrentGroup() {
        guard let deviceName = self.deviceName, let groupName = self.groupName else {
                print("error: ComponentViewController, onComponentRemoveFromCurrentGroup, invalid device name or group name")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Invalid device name or group name is nil.")
                return
        }
        
        MeshFrameworkManager.shared.removeMeshComponent(componentName: deviceName, fromGroup: groupName)  { (networkName: String?, error: Int) in
            guard error == MeshErrorCode.MESH_SUCCESS else {
                print("error: ComponentViewController, onComponentRemoveFromCurrentGroup, invalid device name or group name, error=\(error)")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to remove device:\"\(deviceName)\" from group:\"\(groupName)\". Error Code: \(error)")
                return
            }
            
            // removed success.
            print("ComponentViewController, onComponentRemoveFromCurrentGroup, remove device:\"\(deviceName)\" from group:\"\(groupName)\" message has been sent")
            self.onRemovedFromGroupHandler()
        }
    }
    
    func onRemovedFromGroupHandler() {
        guard let deviceName = self.deviceName, let groupName = self.groupName else {
            print("error: ComponentViewController, onRemovedFromGroupHandler, invalid device name or group name")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Invalid device name or group name.")
            return
        }
        
        componentOperationTimer?.invalidate()
        componentOperationTimer = nil
        indicatorView.stopAnimating()
        operationType = nil
        popoverSelectedGroupName = nil
        if let groups = MeshFrameworkManager.shared.getMeshComponenetGroupList(componentName: deviceName), let _ = groups.filter({$0 == groupName}).first {
            print("error: ComponentViewController, onRemovedFromGroupHandler, failed to remove device:\(deviceName) from group:\(groupName)")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to remove device:\(deviceName) from group:\(groupName)")
        } else {
            print("ComponentViewController, onRemovedFromGroupHandler, Remove device:\(deviceName) from group:\(groupName) success")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Device:\(deviceName) has been removed from group:\(groupName) successfully.", title: "Success")
        }
    }
    
    func onFirmwareOta() {
        print("ComponentViewController, onFirmwareOta, go to device firmware OTA page")
        //UtilityManager.navigateToViewController(sender: self, targetVCClass: DeviceOtaUpgradeViewController.self)
        
        if let vc = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: String(describing: DeviceOtaUpgradeViewController.self)) as? DeviceOtaUpgradeViewController {
            vc.modalPresentationStyle = .fullScreen
            vc.deviceName = self.deviceName
            vc.groupName = self.groupName
            if let deviceName = self.deviceName {
                OtaManager.shared.activeOtaDevice = OtaMeshDevice(meshName: deviceName)
            }
            self.present(vc, animated: true, completion: nil)
        }
    }
}
