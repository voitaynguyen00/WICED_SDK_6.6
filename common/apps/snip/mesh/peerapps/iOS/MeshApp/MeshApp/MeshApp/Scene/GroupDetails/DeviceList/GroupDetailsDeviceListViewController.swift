/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * Group Detail Device List view controller implementation.
 */

import UIKit
import MeshFramework

class GroupDetailsDeviceListViewController: UIViewController {
    @IBOutlet weak var deviceListTableView: UITableView!
    
    var groupName: String?
    var groupComponents: [GroupComponentData]?
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        meshDeviceListInit()
        notificationInit()
        
        deviceListTableView.dataSource = self
        deviceListTableView.delegate = self
        deviceListTableView.separatorStyle = .none
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
        
        NotificationCenter.default.addObserver(self, selector: #selector(self.notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_ON_OFF_STATUS), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(self.notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_LEVEL_STATUS), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(self.notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_HSL_STATUS), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(self.notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_CTL_STATUS), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(self.notificationHandler(_:)),
                                               name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_LIGHTNESS_STATUS), object: nil)
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
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_ON_OFF_STATUS):
            if let onOffStatus = MeshNotificationConstants.getOnOffStatus(userInfo: userInfo),
                let cell = getTableViewCell(by: onOffStatus.deviceName) {
                cell.isOn = onOffStatus.isOn
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_LEVEL_STATUS):
            if let levelStatus = MeshNotificationConstants.getLevelStatus(userInfo: userInfo),
                let cell = getTableViewCell(by: levelStatus.deviceName) {
                cell.isOn = (levelStatus.level > 0) ? true : false
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_HSL_STATUS):
            if let hslStatus = MeshNotificationConstants.getHslStatus(userInfo: userInfo),
                let cell = getTableViewCell(by: hslStatus.deviceName) {
                cell.isOn = (hslStatus.lightness > 0) ? true : false
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_CTL_STATUS):
            if let ctlStatus = MeshNotificationConstants.getCtlStatus(userInfo: userInfo),
                let cell = getTableViewCell(by: ctlStatus.deviceName) {
                cell.isOn = (ctlStatus.targetLightness > 0) ? true : false
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_LIGHTNESS_STATUS):
            if let lightnessStatus = MeshNotificationConstants.getLightnessStatus(userInfo: userInfo),
                let cell = getTableViewCell(by: lightnessStatus.deviceName) {
                cell.isOn = (lightnessStatus.targetLightness > 0) ? true : false
            }
        default:
            break
        }
    }
    
    func meshDeviceListInit() {
        if groupComponents == nil {
            self.groupComponents = []
            if let groupName = UserSettings.shared.currentActiveGroupName,
                let deviceNames = MeshFrameworkManager.shared.getMeshGroupComponents(groupName: groupName) {
                for name in deviceNames {
                    let componentType = MeshFrameworkManager.shared.getMeshComponentType(componentName: name)
                    self.groupComponents!.append(GroupComponentData(name: name, type: componentType))
                }
            }
        }
    }
    
    /**
     * Send command to get devices status.
     * When the device status is reterieved, the value is sent through notification data which are processed in about handlers.
     */
    func getDeviceStatus(name: String?, type: Int) {
        guard let deviceName = name, type != MeshConstants.MESH_COMPONENT_UNKNOWN else {
            return
        }
        
        MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error: Int) in
            guard error == MeshErrorCode.MESH_SUCCESS else {
                self.getDevicestatusAfter(deviceName: deviceName, type: type, milliseconds: 5000)
                return
            }
            
            self.doGetDeviceStatus(deviceName: deviceName, type: type)
        }
    }
    func getDevicestatusAfter(deviceName: String, type: Int, milliseconds: Int) {
        DispatchQueue.main.asyncAfter(deadline: DispatchTime.now() + .milliseconds(milliseconds), execute: {
            MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error: Int) in
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("GroupDetailsDeviceListViewController, getDeviceStatus, failed to connect to the network")
                    return
                }
                
                self.doGetDeviceStatus(deviceName: deviceName, type: type)
            }
        })
    }
    func doGetDeviceStatus(deviceName: String, type: Int) {
        switch type {
        case MeshConstants.MESH_COMPONENT_GENERIC_ON_OFF_CLIENT:
            fallthrough
        case MeshConstants.MESH_COMPONENT_GENERIC_ON_OFF_SERVER:
            let error = MeshFrameworkManager.shared.meshClientOnOffGet(deviceName: deviceName)
            print("GroupDetailsDeviceListViewController, getDeviceStatus, meshClientOnOffGet \(deviceName), error=\(error)")
        case MeshConstants.MESH_COMPONENT_GENERIC_LEVEL_CLIENT:
            fallthrough
        case MeshConstants.MESH_COMPONENT_GENERIC_LEVEL_SERVER:
            let error = MeshFrameworkManager.shared.meshClientLevelGet(deviceName: deviceName)
            print("GroupDetailsDeviceListViewController, getDeviceStatus, meshClientLevelGet \(deviceName), error=\(error)")
        case MeshConstants.MESH_COMPONENT_LIGHT_HSL:
            let error = MeshFrameworkManager.shared.meshClientHslGet(deviceName: deviceName)
            print("GroupDetailsDeviceListViewController, getDeviceStatus, meshClientHslGet \(deviceName), error=\(error)")
        case MeshConstants.MESH_COMPONENT_LIGHT_CTL:
            let error = MeshFrameworkManager.shared.meshClientCtlGet(deviceName: deviceName)
            print("GroupDetailsDeviceListViewController, getDeviceStatus, meshClientCtlGet \(deviceName), error=\(error)")
        case MeshConstants.MESH_COMPONENT_LIGHT_DIMMABLE:
            let error = MeshFrameworkManager.shared.meshClientLightnessGet(deviceName: deviceName)
            print("GroupDetailsDeviceListViewController, getDeviceStatus, meshClientLightnessGet \(deviceName), error=\(error)")
        default:
            break
        }
    }
    
    @IBAction func onSettingBarButtonItemClick(_ sender: UIBarButtonItem) {
    }
}

extension GroupDetailsDeviceListViewController: UITableViewDataSource, UITableViewDelegate {
    func numberOfSections(in tableView: UITableView) -> Int {
        return 1
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        guard let groupComponents = self.groupComponents, groupComponents.count > 0 else {
            return 1
        }
        return groupComponents.count
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        guard let groupComponents = self.groupComponents, groupComponents.count > 0 else {
            // Show empty cell.
            guard let cell = self.deviceListTableView.dequeueReusableCell(withIdentifier: MeshAppStoryBoardIdentifires.DEVICE_LIST_EMPTY_CELL, for: indexPath) as? DeviceListEmptyTableViewCell else {
                return UITableViewCell()
            }
            cell.emptyMessage.text = "No Device Found"
            return cell
        }
        
        guard let cell = self.deviceListTableView.dequeueReusableCell(withIdentifier: MeshAppStoryBoardIdentifires.DEVICE_LIST_CELL, for: indexPath) as? DeviceListTableViewCell else {
            return UITableViewCell()
        }
        
        cell.deviceNameLabel.text = groupComponents[indexPath.row].name
        cell.groupName = groupName
        cell.deviceName = groupComponents[indexPath.row].name
        cell.deviceType = groupComponents[indexPath.row].type
        cell.isOn = false
        // Send async message to get the latest device status, so UI can update and show the real ON/OFF status for the device when the status is updated.
        getDeviceStatus(name: cell.deviceName, type: cell.deviceType)
        return cell
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        guard let groupComponents = self.groupComponents, groupComponents.count > 0 else {
            return
        }
        
        if indexPath.row < groupComponents.count {
            UserSettings.shared.currentActiveGroupName = groupName
            UserSettings.shared.currentActiveComponentName = groupComponents[indexPath.row].name
            print("GroupDetailsDeviceListViewController, didSelectedRowAt=\(indexPath.row), deviceName=\(String(describing: UserSettings.shared.currentActiveComponentName))")
            UtilityManager.navigateToViewController(targetClass: ComponentViewController.self)
        }
    }
    
    func getTableViewCell(by name: String) -> DeviceListTableViewCell? {
        guard let groupComponents = self.groupComponents, groupComponents.count > 0 else {
            return nil
        }
        
        for row in 0..<groupComponents.count {
            let indexPath = IndexPath(row: row, section: 0)
            if let cell = deviceListTableView.cellForRow(at: indexPath) as? DeviceListTableViewCell, let deviceName = cell.deviceName {
                if name == deviceName {
                    return cell
                }
            }
        }
        return nil
    }
}
