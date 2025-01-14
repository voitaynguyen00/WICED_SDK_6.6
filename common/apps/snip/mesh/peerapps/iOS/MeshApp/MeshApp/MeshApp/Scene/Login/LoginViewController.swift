/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * Login view controller implementation.
 */

import UIKit
import MeshFramework

class LoginViewController: UIViewController {
    @IBOutlet weak var accountEmailTextField: UITextField!
    @IBOutlet weak var accountPasswordTextField: UITextField!
    @IBOutlet weak var loginButton: UIButton!
    @IBOutlet weak var signUpButton: UIButton!
    @IBOutlet weak var firmwareUpdateOnlyButton: UIButton!
    @IBOutlet weak var errorInfoView: UIView!
    
    let activeIndcator = CustomIndicatorView()
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        notificationInit()
        errorInfoView.isHidden = true
        accountEmailTextField.delegate = self
        accountPasswordTextField.delegate = self
        let tapBackGround = UITapGestureRecognizer(target: self, action: #selector(onTapBackGround))
        self.view.addGestureRecognizer(tapBackGround)
        
        UserSettings.shared.isAccountLoggedIn = false
        if UserSettings.shared.isAccoutActive, let email = UserSettings.shared.activeEmail, let password = UserSettings.shared.activePssword {
            accountEmailTextField.text = email
            accountPasswordTextField.text = password
            onLoginButtonUp(loginButton)
        } else if !UserSettings.shared.isAccoutActive, let email = UserSettings.shared.activeEmail {
            accountEmailTextField.text = email
        } else if let email = UserSettings.shared.lastActiveAccountEmail {
            accountEmailTextField.text = email
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

    @objc func onTapBackGround() {
        self.view.endEditing(true)
    }

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */
    
    func validateInputAccount() -> Bool {
        if accountEmailTextField.text == nil || accountEmailTextField.text!.isEmpty ||
            !UtilityManager.validateEmail(email: accountEmailTextField.text) {
            errorInfoView.isHidden = false
            return false
        }
        
        if accountPasswordTextField.text == nil || accountPasswordTextField.text!.isEmpty {
            errorInfoView.isHidden = false
            return false
        }
        
        return true
    }

    @IBAction func onLoginButtonUp(_ sender: Any) {
        guard validateInputAccount(), let email = accountEmailTextField.text, let password = accountPasswordTextField.text  else {
            accountPasswordTextField.text = ""
            return
        }

        activeIndcator.showAnimating(parentView: self.view, isTransparent: true)
        DispatchQueue.main.async { [weak self] in
            NetworkManager.shared.accountLogin(name: email, password: password) { (status: Int) -> Void in
                self?.activeIndcator.stopAnimating()
                
                if status != 0 {
                    self?.accountPasswordTextField.text = ""
                    self?.errorInfoView.isHidden = false
                    print("error: onLoginButtonUp, NetworkManager.shared.accountLogin status=\(status)")
                    return
                }
                
                print("onLoginButtonUp, success")
                UserSettings.shared.activeEmail = email
                UserSettings.shared.activeName = UserSettings.shared.accountsName["email"]
                UserSettings.shared.activePssword = password
                UserSettings.shared.isAccoutActive = true                
                UtilityManager.navigateToViewController(targetClass: StartViewController.self)
            }
        }
    }
    
    @IBAction func onFirmwareUpdateOnlyButtonUp(_ sender: Any) {
    }
}

extension LoginViewController: UITextFieldDelegate {
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if textField == accountEmailTextField {
            _ = accountPasswordTextField.becomeFirstResponder()
        } else if textField == accountPasswordTextField {
            textField.resignFirstResponder()
            onLoginButtonUp(loginButton)
        }
        return true
    }
    
    func textField(_ textField: UITextField, shouldChangeCharactersIn range: NSRange, replacementString string: String) -> Bool {
        if !errorInfoView.isHidden {
            errorInfoView.isHidden = true
        }
        return true
    }
}
