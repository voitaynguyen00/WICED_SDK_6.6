/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * Group Detail view controller implementation.
 */

import UIKit
import MeshFramework

struct GroupComponentData {
    var name: String
    var type: Int
}

enum GroupDetailsPopoverChoices: String {
    case rename = "Rename Group"
    case delete = "Delete Group"
    
    static var allValues = [GroupDetailsPopoverChoices.rename.rawValue,
                            GroupDetailsPopoverChoices.delete.rawValue]
}

class GroupDetailsViewController: UIViewController {
    @IBOutlet weak var topNavigationBar: UINavigationItem!
    @IBOutlet weak var backBarButtonItem: UIBarButtonItem!
    @IBOutlet weak var settingsBarButtonItem: UIBarButtonItem!
    @IBOutlet weak var groupNameLabel: UILabel!
    @IBOutlet weak var AddDeviceButton: CustomRoundedRectButton!
    @IBOutlet weak var segmentedControl: UISegmentedControl!
    @IBOutlet weak var contentView: UIView!
    
    private let indicatorView = CustomIndicatorView()
    
    var currentSegmentedSelectedIndex = MeshAppConstants.GROUPT_DETAIL_SEGMENTED_DEFAULT
    var currentContentViewController: UIViewController?
    
    private var mGroupName: String?
    var groupName: String? {
        get { return mGroupName }
        set(value) {
            mGroupName = value
            groupNameLabel.text = mGroupName
            if mGroupName == MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME {
                // Not allowed to delete or rename the default group.
                settingsBarButtonItem.isEnabled = false
            } else {
                settingsBarButtonItem.isEnabled = true
            }
            meshGroupInit()
            segmentedControlInit()
        }
    }
    var groupComponents: [GroupComponentData] = []
    
    lazy var controlsVC: UIViewController = {
        guard let groupControlsVC = self.storyboard?.instantiateViewController(withIdentifier: MeshAppStoryBoardIdentifires.GROUP_DETAILS_CONTROLS) as? GroupDetailsControlsViewController else {
            print("error: GroupDetailsViewController, failed to load \(MeshAppStoryBoardIdentifires.GROUP_DETAILS_CONTROLS) ViewController from Main storyboard")
            return UIViewController()
        }
        return groupControlsVC as UIViewController
    }()
    
    lazy var deviceListVC: UIViewController = {
        guard let groupDeviceListVC = self.storyboard?.instantiateViewController(withIdentifier: MeshAppStoryBoardIdentifires.GROUP_DETAILS_DEVICE_LIST) as? GroupDetailsDeviceListViewController else {
            print("error: GroupDetailsViewController, failed to load \(MeshAppStoryBoardIdentifires.GROUP_DETAILS_DEVICE_LIST) ViewController from Main storyboard")
            return UIViewController()
        }
        return groupDeviceListVC as UIViewController
    }()
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        notificationInit()
        if let _ = self.groupName {
            meshGroupInit()
            segmentedControlInit()
        } else {
            self.groupName = UserSettings.shared.currentActiveGroupName
        }
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
        default:
            break
        }
    }
    
    func meshGroupInit() {
        guard let groupName = self.groupName else {
            print("error: GroupDetailsViewController, meshGroupInit, invalid groupName nil")
            return  // Must wait the the groupName is real set, then do the initialization.
        }

        self.groupComponents.removeAll()
        var componets: [String]?
        if groupName == MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME {
            componets = MeshFrameworkManager.shared.getlAllMeshGroupComponents(groupName: groupName)
        } else {
            componets = MeshFrameworkManager.shared.getMeshGroupComponents(groupName: groupName)
        }
        if let deviceNames = componets {
            for name in deviceNames {
                let componentType = MeshFrameworkManager.shared.getMeshComponentType(componentName: name)
                self.groupComponents.append(GroupComponentData(name: name, type: componentType))
                print("GroupDetailsViewController, group: \(groupName) has component: name=\(name), type=\(componentType)")
            }
        }
    }

    func segmentedControlInit() {
        var index: Int = self.currentSegmentedSelectedIndex
        if groupComponents.count == 0 {
            index = MeshAppConstants.GROUPT_DETAIL_SEGMENTED_ALL_DEVICE
        }
        if self.currentSegmentedSelectedIndex >= self.segmentedControl.numberOfSegments {
            index = self.segmentedControl.numberOfSegments - 1
        } else if self.currentSegmentedSelectedIndex < 0 {
            index = MeshAppConstants.GROUPT_DETAIL_SEGMENTED_DEFAULT
        }
        self.currentSegmentedSelectedIndex = index
        self.segmentedControl.selectedSegmentIndex = index
        self.segmentedControl.setTitle("Devices (\(self.groupComponents.count))", forSegmentAt: MeshAppConstants.GROUPT_DETAIL_SEGMENTED_ALL_DEVICE)
        self.segmentedControl.addTarget(self, action: #selector(onSegmentedControlValueChanged(_:)), for: .valueChanged)
        doSegmentedControlValueChanged(selectedSegmentIndex: self.segmentedControl.selectedSegmentIndex)
    }
    
    func doSegmentedControlValueChanged(selectedSegmentIndex: Int) {
        self.currentSegmentedSelectedIndex = selectedSegmentIndex
        if self.segmentedControl.selectedSegmentIndex > 0 {
            // show Group Details Device List content.
            print("GroupDetailsViewController, content of group Controls selected")
            if let deviceListVC = self.deviceListVC as? GroupDetailsDeviceListViewController {
                deviceListVC.groupName = self.groupName
                deviceListVC.groupComponents = self.groupComponents
            }
            displaySegmentedControlContent(for: self.deviceListVC)
        } else {
            // show Group Details Controls content.
            print("GroupDetailsViewController, content of group Device List selected")
            
            if let controlVC = self.controlsVC as? GroupDetailsControlsViewController {
                controlVC.isDeviceControl = false
                controlVC.deviceName = self.groupName
                controlVC.groupComponents = self.groupComponents
            }
            displaySegmentedControlContent(for: self.controlsVC)
        }
    }
    
    func displaySegmentedControlContent(for vc: UIViewController) {
        self.currentContentViewController?.view.removeFromSuperview()
        self.currentContentViewController?.removeFromParent()
        self.currentContentViewController = nil
        
        self.addChild(vc)
        vc.didMove(toParent: self)
        vc.view.frame = self.contentView.bounds
        self.contentView.addSubview(vc.view)
        self.currentContentViewController = vc
    }
    
    @objc func onSegmentedControlValueChanged(_ sender: UISegmentedControl) {
        doSegmentedControlValueChanged(selectedSegmentIndex: self.segmentedControl.selectedSegmentIndex)
    }

    @IBAction func onBackBarButtonItemClick(_ sender: UIBarButtonItem) {
        print("GroupDetailsViewController, onBackBarButtonItemClick, go back to network list")
        // Do nothing, controlled by segue.
    }
    
    @IBAction func onSettingsBarButtonItemClick(_ sender: UIBarButtonItem) {
        print("GroupDetailsViewController, onSettingsBarButtonItemClick")
        let choices = GroupDetailsPopoverChoices.allValues
        let controller = PopoverChoiceTableViewController(choices: choices) { (index: Int, selection: String) in
            print("GroupDetailsViewController, onSettingsBarButtonItemClick, index=\(index), selection=\(selection)")
            guard let choice = GroupDetailsPopoverChoices.init(rawValue: selection) else { return }
            
            switch choice {
            case .rename:
                self.onGroupRename()
            case .delete:
                self.onGroupDelete()
            }
        }
        controller.preferredContentSize = CGSize(width: 190, height: controller.getPreferredPopoverViewSize())
        controller.showPopoverPresentation(parent: self, sourceView: sender.value(forKey: "view") as? UIView)
    }
    
    @IBAction func onAddDeviceButtonClick(_ sender: CustomRoundedRectButton) {
        print("GroupDetailsViewController, onAddDeviceButtonClick")
    }
    
    // MARK: - Navigation
    
    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
        if let identifier = segue.identifier {
            print("GroupDetailsViewController, segue.identifier=\(identifier)")
            switch identifier {
            case MeshAppStoryBoardIdentifires.SEGUE_GROUP_DETAIL_TO_ADD_DEVICE:
                if let unprovisionedDevicesVC = segue.destination as? UnprovisionedDevicesViewController {
                    unprovisionedDevicesVC.groupName = self.groupName
                }
            default:
                break
            }
        }
    }
    
    func onGroupRename() {
        print("GroupDetailsViewController, onGroupRename, show input new group dialogue")
        let alertController = UIAlertController(title: GroupDetailsPopoverChoices.rename.rawValue, message: nil, preferredStyle: .alert)
        alertController.addTextField { (textField: UITextField) in
            textField.placeholder = "Enter New Group Name"
        }
        alertController.addAction(UIAlertAction(title: "Cancel", style: .default, handler: nil))
        alertController.addAction(UIAlertAction(title: "Confirm", style: .default, handler: { (action: UIAlertAction) -> Void in
            if let textField = alertController.textFields?.first, let newGroupName = textField.text, newGroupName.count > 0,
                let oldGroupName = UserSettings.shared.currentActiveGroupName {
                self.indicatorView.showAnimating(parentView: self.view)
                MeshFrameworkManager.shared.meshClientRename(oldName: oldGroupName, newName: newGroupName, completion: { (networkName: String?, error: Int) in
                    self.indicatorView.stopAnimating()
                    guard networkName == UserSettings.shared.currentActiveMeshNetworkName, error == MeshErrorCode.MESH_SUCCESS else {
                        print("error: GroupDetailsViewController, failed to call meshClientRename with oldGroupName:\(oldGroupName), newGroupName=\(newGroupName), error=\(error)")
                        UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to rename \"\(oldGroupName)\" mesh group. Error Code: \(error)")
                        return
                    }
                    
                    print("GroupDetailsViewController, rename \"\(oldGroupName)\" group to new name=\"\(newGroupName)\" success")
                    // update GUI and instance data.
                    self.groupName = newGroupName
                    UserSettings.shared.currentActiveGroupName = newGroupName
                    self.groupNameLabel.text = newGroupName
                    // update group status values in UserSettings
                    if let values = UserSettings.shared.getComponentStatus(componentName: oldGroupName) {
                        UserSettings.shared.removeComponentStatus(componentName: oldGroupName)
                        UserSettings.shared.setComponentStatus(componentName: newGroupName, values: values)
                    }
                    
                    UtilityManager.showAlertDialogue(parentVC: self, message: "Group name has been changed to \"\(newGroupName)\" success.", title: "Success")
                })
            } else {
                UtilityManager.showAlertDialogue(parentVC: self, message: "Empty or Invalid new group name!", title: "Error")
            }
        }))
        self.present(alertController, animated: true, completion: nil)
    }
    
    func onGroupDelete() {
        print("GroupDetailsViewController, onGroupDelete")
        guard let currentGroupName = UserSettings.shared.currentActiveGroupName else {
            print("error: ComponentViewController, invalid group name, failed to delete group:\(String(describing: UserSettings.shared.currentActiveGroupName))")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to delete the group, invalid group name!", title: "Error")
            return
        }
        // The default group is not allowed to be deleted, it's aimed to demo the function to control all devices that added to the mesh network whatever it in any groups.
        guard currentGroupName != MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME else {
            print("error: ComponentViewController, not allowed to default all components group: \(MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME).")
            UtilityManager.showAlertDialogue(parentVC: self, message: "Not allowed to delete the default group \"\(MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME)\" which including all provisioned components. If you want to delete all devices in the mesh network, please try to delete the mesh network instead.", title: "Error")
            return
        }
        
        if let componets = MeshFrameworkManager.shared.getMeshGroupComponents(groupName: currentGroupName) {
            // still have some conponents not deleted from the group, prompt user relative information, and should select cancal or continue.
            UtilityManager.showAlertDialogue(
                parentVC: self,
                message: "Are you sure you want to delete the \"\(currentGroupName)\" group which still have \(componets.count) devices not deleted yet. If continue delete this group will move all undeleted devices into the \"\(MeshAppConstants.MESH_DEFAULT_ALL_COMPONENTS_GROUP_NAME)\" group?\n\nClick \"OK\" button to continue.\nClick \"Cancel\" button to exit",
                title: "Warning",
                cancelHandler: { (action: UIAlertAction) in return },
                okayHandler: { (action: UIAlertAction) in self.doDeleteGroup(groupName: currentGroupName) }
            )
        } else {
            // no component in the group, delete the group directly.
            _doDeleteGroup(groupName: currentGroupName)
        }
    }
    
    func _doDeleteGroup(groupName: String) {
        MeshFrameworkManager.shared.deleteMeshGroup(groupName: groupName) { (networkName: String?, error: Int) in
            self.indicatorView.stopAnimating()
            guard networkName == UserSettings.shared.currentActiveMeshNetworkName, error == MeshErrorCode.MESH_SUCCESS else {
                print("error: ComponentViewController, failed to delete group:\(groupName)")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to delete group \"\(groupName)\". Error Code: \(error).", title: "Error")
                return
            }
            
            print("ComponentViewController, delete group:\(groupName) success")
            // update group status values in UserSettings
            UserSettings.shared.removeComponentStatus(componentName: groupName)
            
            // go back to group list scense.
            UtilityManager.navigateToViewController(targetClass: GroupListViewController.self)
        }
    }
        
    func doDeleteGroup(groupName: String) {
        indicatorView.showAnimating(parentView: self.view)
        MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error) in
            guard error == MeshErrorCode.MESH_SUCCESS else {
                self.indicatorView.stopAnimating()
                print("GroupDetailsViewController, onGroupDelete, unable to connect to the mesh network")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to connect to mesh network, unable to delete the group.")
                return
            }
            
            self._doDeleteGroup(groupName: groupName)
        }
    }
}
