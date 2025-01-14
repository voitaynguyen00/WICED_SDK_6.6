/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * Device List table view cell implementation.
 */

import UIKit
import MeshFramework

class DeviceListTableViewCell: UITableViewCell {
    @IBOutlet weak var deviceIconImage: UIImageView!
    @IBOutlet weak var deviceNameLabel: UILabel!
    @IBOutlet weak var controlsButton: UIButton!
    @IBOutlet weak var turnOnButton: UIButton!
    @IBOutlet weak var turnOffButton: UIButton!

    var groupName: String?
    var deviceName: String?
    var deviceType: Int = MeshConstants.MESH_COMPONENT_UNKNOWN
    var isOn: Bool = false {
        didSet {
            switch deviceType {
            case MeshConstants.MESH_COMPONENT_GENERIC_ON_OFF_CLIENT:
                fallthrough
            case MeshConstants.MESH_COMPONENT_GENERIC_ON_OFF_SERVER:
                // TODO[optional]: use the special images for ON/OFF devices if have.
                deviceIconImage.image = UIImage(named: (isOn ? MeshAppImageNames.lightOnImage : MeshAppImageNames.lightOffImage))
            case MeshConstants.MESH_COMPONENT_GENERIC_LEVEL_CLIENT:
                fallthrough
            case MeshConstants.MESH_COMPONENT_GENERIC_LEVEL_SERVER:
                // TODO[optional]: use the special images for level devices if have.
                deviceIconImage.image = UIImage(named: (isOn ? MeshAppImageNames.lightOnImage : MeshAppImageNames.lightOffImage))
            case MeshConstants.MESH_COMPONENT_LIGHT_HSL:
                // TODO[optional]: use the special images for HSL light devices if have.
                deviceIconImage.image = UIImage(named: (isOn ? MeshAppImageNames.lightOnImage : MeshAppImageNames.lightOffImage))
            case MeshConstants.MESH_COMPONENT_LIGHT_CTL:
                // TODO[optional]: use the special images for CTL light devices if have.
                deviceIconImage.image = UIImage(named: (isOn ? MeshAppImageNames.lightOnImage : MeshAppImageNames.lightOffImage))
            case MeshConstants.MESH_COMPONENT_LIGHT_DIMMABLE:
                // TODO[optional]: use the special images for lightness light devices if have.
                deviceIconImage.image = UIImage(named: (isOn ? MeshAppImageNames.lightOnImage : MeshAppImageNames.lightOffImage))
            case MeshConstants.MESH_COMPONENT_SENSOR_SERVER:
                fallthrough
            case MeshConstants.MESH_COMPONENT_SENSOR_CLIENT:
                // TODO[optional]: use the special images for lightness light devices if have.
                deviceIconImage.image = UIImage(named: MeshAppImageNames.sensorImage)
            default:
                // TODO[optional]: use the special images for unknown type devices if have.
                deviceIconImage.image = UIImage(named: (isOn ? MeshAppImageNames.lightOnImage : MeshAppImageNames.lightOffImage))
                break
            }
        }
    }
    
    override func awakeFromNib() {
        super.awakeFromNib()
        // Initialization code
    }

    override func setSelected(_ selected: Bool, animated: Bool) {
        //super.setSelected(selected, animated: animated)

        // Configure the view for the selected state
    }

    @IBAction func onControlButtonClick(_ sender: UIButton) {
        print("DeviceListTableViewCell, onControlButtonClick")
        if let name = self.deviceNameLabel.text, name.count > 0 {
            UserSettings.shared.currentActiveGroupName = groupName
            UserSettings.shared.currentActiveComponentName = deviceName
            UtilityManager.navigateToViewController(targetClass: ComponentViewController.self)
        }
    }
    
    @IBAction func onTurnOnButtonClick(_ sender: UIButton) {
        print("DeviceListTableViewCell, onTurnOnButtonClick, deviceName=\(String(describing: deviceName))")
        turnDeviceOnOff(deviceName: deviceName, isOn: true)
    }
    
    @IBAction func onTurnOffButtonClick(_ sender: UIButton) {
        print("DeviceListTableViewCell, onTurnOffButtonClick, deviceName=\(String(describing: deviceName))")
        turnDeviceOnOff(deviceName: deviceName, isOn: false)
    }
    
    /**
     Turn the remote device ON or OFF.
     
     @param deviceName      Name of the target mesh device to turn ON or OFF.
     @param isON            Indicate the operation of turn ON (turn) or turn OFF (false).
     @param reliable        Indicates to send using the Acknowledged (true) or the Unacknowledged (false) message.
                            For mesh device, it's set to true by default.
     
     @return                None.
     
     Note, when the reliable set to true, the MeshNotificationConstants.MESH_CLIENT_ON_OFF_STATUS event will be sent to GroupDetailsDeviceListViewController instance,
     so, the device status will be updated and shown in the UI.
     */
    func turnDeviceOnOff(deviceName: String?, isOn: Bool, reliable: Bool = true) {
        if let deviceName = deviceName {
            MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error) in
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("error: DeviceListTableViewCell, turnDeviceOnOff(deviceName:\(deviceName), isOn:\(isOn)), failed to connect to the mesh network")
                    return
                }

                let error = MeshFrameworkManager.shared.meshClientOnOffSet(deviceName: deviceName, isOn: isOn, reliable: reliable)
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("error: DeviceListTableViewCell, meshClientOnOffSet(deviceName:\(deviceName), isOn:\(isOn), reliable=\(reliable)) failed, error=\(error)")
                    return
                }
                print("DeviceListTableViewCell, meshClientOnOffSet(deviceName:\(deviceName), isOn:\(isOn), reliable=\(reliable)) message sent out success")
            }
        }
    }
}
