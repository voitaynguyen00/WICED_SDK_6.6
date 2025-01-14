/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * Network list view controller implementation.
 */

import UIKit
import MeshFramework

enum NetworkListPopoverChoices: String {
    case closeNetwork = "Close Network"
    case deleteNetwork = "Delete Network"
    case importNetwork = "Import Network"
    case exportNetwork = "Export Network"
    case downloadNetworks = "Download Networks"
    
    static var allValues = [NetworkListPopoverChoices.closeNetwork.rawValue,
                            NetworkListPopoverChoices.deleteNetwork.rawValue,
                            NetworkListPopoverChoices.importNetwork.rawValue,
                            NetworkListPopoverChoices.exportNetwork.rawValue,
                            NetworkListPopoverChoices.downloadNetworks.rawValue]
}

class NetworkListViewController: UIViewController {
    @IBOutlet weak var titleNavigationBar: UINavigationItem!
    @IBOutlet weak var leftMenuButton: UIBarButtonItem!
    @IBOutlet weak var moreOperationListButton: UIBarButtonItem!
    @IBOutlet weak var pageNameLable: UILabel!
    @IBOutlet weak var addNetworkButton: CustomRoundedRectButton!
    @IBOutlet weak var networkListTableview: UITableView!
    
    private let indicatorView = CustomIndicatorView()
    
    private var meshNetworks: [String] = []
    private var selectedRadio: Int = -1     // -1 indicates not selected yet.
    
    func updateNetworkListCount() {
        pageNameLable.text = (meshNetworks.count > 0) ? "My Networks (\(meshNetworks.count))" : "My Networks"
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Do any additional setup after loading the view.
        meshNetworkListInit()
        notificationInit()
        
        titleNavigationBar.title = UserSettings.shared.activeName ?? ""
        updateNetworkListCount()
        networkListTableview.dataSource = self
        networkListTableview.delegate = self
        networkListTableview.separatorStyle = .none
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
    
    func meshNetworkListInit() {
        if let networks = MeshFrameworkManager.shared.getAllMeshNetworks(), networks.count > 0 {
            meshNetworks = networks
            UserSettings.shared.meshNetworks = meshNetworks
        }
    }
    
    @IBAction func onAddNetworkButtonUp(_ sender: UIButton) {
        print("NetworkListViewController, onAddNetworkButtonUp")
        let alertController = UIAlertController(title: "Add Network", message: nil, preferredStyle: .alert)
        alertController.addTextField { (textField: UITextField) in
            textField.placeholder = "New Network Name"
        }
        alertController.addAction(UIAlertAction(title: "Cancel", style: .default, handler: nil))
        alertController.addAction(UIAlertAction(title: "Confirm", style: .default, handler: { (action: UIAlertAction) -> Void in
            if let textField = alertController.textFields?.first, let newNetworkName = textField.text, newNetworkName.count > 0 {
                let error = MeshFrameworkManager.shared.createMeshNetwork(provisioinerName: UserSettings.shared.provisionerName, networkName: newNetworkName)
                if error != MeshErrorCode.MESH_SUCCESS {
                    print("NetworkListViewController, failed to createMeshNetwork with name=\"\(newNetworkName)\"")
                    UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to Create Network with name: \"\(newNetworkName)\". Error Code: \(error)")
                } else {
                    print("NetworkListViewController, createMeshNetwork with name=\"\(newNetworkName)\" success")
                    self.meshNetworkListInit()
                    self.updateNetworkListCount()
                    self.networkListTableview.reloadData()
                }
            } else {
                UtilityManager.showAlertDialogue(parentVC: self, message: "Empty or Invalid Network Name!", title: "Error")
            }
        }))
        self.present(alertController, animated: true, completion: nil)
    }
    
    @IBAction func onNetworkListMenuButtonUp(_ sender: UIBarButtonItem) {
        print("NetworkListViewController, onNetworkListMenuButtonUp")
        self.definesPresentationContext = true
        UtilityManager.navigateToViewController(sender: self, targetVCClass: MenuViewController.self, modalPresentationStyle: UIModalPresentationStyle.overCurrentContext)
    }
    
    @IBAction func onNetworkListMoreButtonUp(_ sender: UIBarButtonItem) {
        print("NetworkListViewController, onNetworkListMoreButtonUp")
        let choices = NetworkListPopoverChoices.allValues
        let controller = PopoverChoiceTableViewController(choices: choices) { (index: Int, selection: String) in
            print("NetworkListViewController, onNetworkListMoreButtonUp, index=\(index), selection=\(selection)")
            guard let choice = NetworkListPopoverChoices.init(rawValue: selection) else { return }
            
            switch choice {
            case .closeNetwork:
                self.onCloseNetwork()
            case .deleteNetwork:
                self.onDeleteNetwork()
            case .downloadNetworks:
                self.onDownloadNetworks()
            case .exportNetwork:
                self.onExportNetwork()
            case .importNetwork:
                self.onImportNetwork()
            }
        }
        controller.preferredContentSize = CGSize(width: 190, height: controller.getPreferredPopoverViewSize())
        controller.showPopoverPresentation(parent: self, sourceView: sender.value(forKey: "view") as? UIView)
    }

    func doMeshNetworkOpen(provisionerName: String, networkName: String) {
        print("NetworkListViewController, openMeshNetwork, provisionerName=\(UserSettings.shared.provisionerName), networkName=\(networkName)")
        if provisionerName == MeshFrameworkManager.shared.getOpenedMeshProvisionerName(), networkName == MeshFrameworkManager.shared.getOpenedMeshNetworkName() {
            // The mesh network has already been opened, return earily.
            print("NetworkListViewController, openMeshNetwork, networkName=\(networkName) already opened, return early.")
            UserSettings.shared.isCurrentActiveMeshNetworkOpenned = true
            UtilityManager.navigateToViewController(targetClass: GroupListViewController.self)
            return
        } else if let _ = MeshFrameworkManager.shared.getOpenedMeshNetworkName() {
            // Currenlty, mesh client has already opened with another mesh network, so should disconnect and close it firstly.
            self.indicatorView.showAnimating(parentView: self.view, isTransparent: true)
            MeshFrameworkManager.shared.disconnectMeshNetwork { (isConnected: Bool, connId: Int, addr: Int, isOverGatt: Bool, error: Int) in
                print("isConnected=\(isConnected), error=\(error)")
                MeshFrameworkManager.shared.closeMeshNetwork()
                
                self.tryToOpenMeshNetwork(provisionerName: provisionerName, openNetworkName: networkName)
            }
            return
        }
        
        self.tryToOpenMeshNetwork(provisionerName: provisionerName, openNetworkName: networkName)
    }
    
    func tryToOpenMeshNetwork(provisionerName: String, openNetworkName: String) {
        self.indicatorView.showAnimating(parentView: self.view, isTransparent: true)
        MeshFrameworkManager.shared.openMeshNetwork(provisioinerName: provisionerName, networkName: openNetworkName) { (_ networkName: String?, _ status: Int, _ error: Int) in
            self.indicatorView.stopAnimating()
            guard error == MeshErrorCode.MESH_SUCCESS else {
                print("error: NetworkListViewController, failed to openMeshNetwork, error=\(error)")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to open mesh mesh work: \(openNetworkName). Error Code: \(error)")
                return
            }
            guard status == MeshErrorCode.MESH_SUCCESS else {
                print("error: NetworkListViewController, failed to openMeshNetwork, completion with status=\(status)")
                UtilityManager.showAlertDialogue(parentVC: self, message: "Failed to open mesh mesh work: \(openNetworkName). Status: \(status)")
                return
            }
            
            print("NetworkListViewController, onOpenNetwork, mesh network: \(openNetworkName) opened success")
            // TransitionViewController will monitor on the open comletion callack and do navigation on the open completion status.
        }
        
        // Jump to TransitiionViewController when opening the network, after openned the network success, then go to GroupListViewController.
        print("NetworkListViewController, onOpenNetwork, opening mesh network: \(openNetworkName), waiting open completion callback")
        UtilityManager.navigateToViewController(targetClass: TransitionViewController.self)
    }
    
    func onCloseNetwork() {
        MeshFrameworkManager.shared.disconnectMeshNetwork { (isConnected: Bool, connId: Int, addr: Int, isOverGatt: Bool, error: Int) in
            MeshFrameworkManager.shared.closeMeshNetwork()
            
            DispatchQueue.main.async {
                for index in 0..<self.meshNetworks.count {
                    let indexPath = IndexPath(row: index, section: 0)
                    if let cell = self.networkListTableview.cellForRow(at: indexPath) as? NetworkListTableViewCell {
                        cell.networkOpenStatusSwitch.setOn(false, animated: true)
                    }
                }
                UserSettings.shared.currentActiveMeshNetworkName = nil
                UserSettings.shared.isCurrentActiveMeshNetworkOpenned = false
            }
        }
    }
    
    func onDeleteNetwork() {
        SelectNetworkPopoverViewController.popoverAction = .deleteNetwork
        if let deleteSelectPopoverViewController = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: MeshAppStoryBoardIdentifires.NETWORK_LIST_SELECT_POPOVER) as? SelectNetworkPopoverViewController {
            deleteSelectPopoverViewController.modalPresentationStyle = .custom
            self.present(deleteSelectPopoverViewController, animated: true, completion: nil)
        }
    }
    
    func onDownloadNetworks() {
        self.indicatorView.showAnimating(parentView: self.view, isTransparent: false)
        DispatchQueue.main.async {
            NetworkManager.shared.restoreMeshFiles { (status) in
                print("NetworkListViewController, onDownloadNetworks, restoreMeshFiles status=\(status)")
                self.indicatorView.stopAnimating()
            }
        }
    }
    
    func onExportNetwork() {
        SelectNetworkPopoverViewController.popoverAction = .exportNetwork
        if let exportSelectPopoverViewController = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: MeshAppStoryBoardIdentifires.NETWORK_LIST_SELECT_POPOVER) as? SelectNetworkPopoverViewController {
            exportSelectPopoverViewController.modalPresentationStyle = .custom
            self.present(exportSelectPopoverViewController, animated: true, completion: nil)
        }
    }
    
    // Currently, as an example, only support importing networks under the exported networks storage.
    func onImportNetwork() {
        guard let networks = MeshFrameworkManager.shared.meshNetworksUnderExportedMeshStorage else {
            print("NetworkListViewController, onImportNetwork, not networks found under \(MeshFrameworkManager.shared.defaultExportStorageFolderName)")
            UtilityManager.showAlertDialogue(parentVC: self, message: "No network found for importing", title: "Warnning")
            return
        }
        print("NetworkListViewController, onImportNetwork, networks=\(networks)")
        
        SelectNetworkPopoverViewController.popoverAction = .importNetwork
        UtilityManager.navigateToViewController(sender: self, targetVCClass: SelectNetworkPopoverViewController.self, modalPresentationStyle: .custom)
    }
}

extension NetworkListViewController: UITableViewDataSource, UITableViewDelegate {
    func numberOfSections(in tableView: UITableView) -> Int {
        return 1
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        if meshNetworks.count == 0 {
            return 1
        }
        return meshNetworks.count
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        if meshNetworks.count == 0 {
            // Show empty cell.
            guard let cell = self.networkListTableview.dequeueReusableCell(withIdentifier: MeshAppStoryBoardIdentifires.NETWORK_LIST_EMPTY_CELL, for: indexPath) as? NetworkListEmptyTableViewCell else {
                return UITableViewCell()
            }
            cell.emptyMessageLabel.text = "Your network seems empty! Try adding a network."
            return cell
        }
        
        guard let cell = self.networkListTableview.dequeueReusableCell(withIdentifier: MeshAppStoryBoardIdentifires.NETWORK_LIST_CELL, for: indexPath) as? NetworkListTableViewCell else {
            return UITableViewCell()
        }
        
        cell.networkName = meshNetworks[indexPath.row]
        return cell
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        if meshNetworks.count == 0, indexPath.row >= meshNetworks.count {
            return
        }
        
        let networkName = meshNetworks[indexPath.row]
        if let cell = networkListTableview.cellForRow(at: indexPath) as? NetworkListTableViewCell, !networkName.isEmpty {
            cell.networkName = networkName
            UserSettings.shared.currentActiveMeshNetworkName = networkName
            UserSettings.shared.isCurrentActiveMeshNetworkOpenned = false
            doMeshNetworkOpen(provisionerName: UserSettings.shared.provisionerName, networkName: networkName)
        } else {
            print("error: NetworkListViewController, failed to didSelectRowAt:\(indexPath.row), networkName;\(networkName)")
        }
    }
}
