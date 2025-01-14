/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * Group Detail Controls view controller implementation.
 */

import UIKit
import MeshFramework

// Note, the MeshControlPickerType Type should be mapped to the component type.
// See MeshConstants.MESH_COMPONENT_* for all supported component type.
enum MeshControlPickerType: String {
    case GENERIC_ON_OFF = "Generic OnOff"
    case GENERIC_LEVEL = "Generic Level"
    case LIGHT_DIMMABLE = "lightness"
    case POWER_OUTLET = "Outlet"
    case LIGHT_HSL = "HSL"
    case LIGHT_CTL = "CTL"
    case LIGHT_XYL = "XYL"
    case VENDOR_SPECIFIC = "Vendor"
    
    // When other component types are supported, add the supported picker type in the all values list.
    static var allValues = [MeshControlPickerType.LIGHT_HSL.rawValue,
                            MeshControlPickerType.LIGHT_CTL.rawValue,
                            MeshControlPickerType.LIGHT_DIMMABLE.rawValue]
    
    static func convertPickerTypeToMeshComponentType(pickerType: MeshControlPickerType) -> Int {
        switch pickerType {
        case .GENERIC_ON_OFF:
            return MeshConstants.MESH_COMPONENT_GENERIC_ON_OFF_SERVER
        case .GENERIC_LEVEL:
            return MeshConstants.MESH_COMPONENT_GENERIC_LEVEL_SERVER
        case .LIGHT_HSL:
            return MeshConstants.MESH_COMPONENT_LIGHT_HSL
        case .LIGHT_CTL:
            return MeshConstants.MESH_COMPONENT_LIGHT_CTL
        case LIGHT_DIMMABLE:
            return MeshConstants.MESH_COMPONENT_LIGHT_DIMMABLE
        case POWER_OUTLET:
            return MeshConstants.MESH_COMPONENT_POWER_OUTLET
        case LIGHT_XYL:
            return MeshConstants.MESH_COMPONENT_LIGHT_XYL
        case VENDOR_SPECIFIC:
            return MeshConstants.MESH_COMPONENT_VENDOR_SPECIFIC
        }
    }
    
    static func convertMeshComponentTypeToPickerType(componentType: Int) -> MeshControlPickerType {
        switch componentType {
        case MeshConstants.MESH_COMPONENT_GENERIC_ON_OFF_CLIENT:
            fallthrough
        case MeshConstants.MESH_COMPONENT_GENERIC_ON_OFF_SERVER:
            return .GENERIC_ON_OFF
        case MeshConstants.MESH_COMPONENT_GENERIC_LEVEL_CLIENT:
            fallthrough
        case MeshConstants.MESH_COMPONENT_GENERIC_LEVEL_SERVER:
            return .GENERIC_LEVEL
        case MeshConstants.MESH_COMPONENT_LIGHT_HSL:
            return .LIGHT_HSL
        case MeshConstants.MESH_COMPONENT_LIGHT_CTL:
            return .LIGHT_CTL
        case MeshConstants.MESH_COMPONENT_LIGHT_DIMMABLE:
            return .LIGHT_DIMMABLE
        case MeshConstants.MESH_COMPONENT_POWER_OUTLET:
            return .POWER_OUTLET
        case MeshConstants.MESH_COMPONENT_LIGHT_XYL:
            return .LIGHT_XYL
        case MeshConstants.MESH_COMPONENT_VENDOR_SPECIFIC:
            return .VENDOR_SPECIFIC
        default:
            return .LIGHT_HSL
        }
    }
}

class GroupDetailsControlsViewController: UIViewController {
    @IBOutlet weak var nameView: UIView!
    @IBOutlet weak var nameViewHeightConstraint: NSLayoutConstraint!
    @IBOutlet weak var nameLabel: UILabel!
    @IBOutlet weak var controlTypeButton: UIButton!
    @IBOutlet weak var pickerView: UIView!
    @IBOutlet weak var pickerImage: UIImageView!
    @IBOutlet weak var sliderGroupView: UIView!
    @IBOutlet weak var sliderPercentageLabel: UILabel!
    @IBOutlet weak var brightnessSlider: UISlider!
    @IBOutlet weak var onOffButtonsView: UIView!
    @IBOutlet weak var turnOnButton: UIButton!
    @IBOutlet weak var trunOffButton: UIButton!
    @IBOutlet weak var contentScrollView: UIScrollView!
    @IBOutlet weak var deviceImageIcon: UIImageView!
    
    var isDeviceControl: Bool = false
    var groupName: String?
    var deviceName: String? {    // it can be group name or component device name.
        didSet {
            if nameLabel != nil {
                nameLabel.text = deviceName ?? MeshConstantText.UNKNOWN_DEVICE_NAME
            }
        }
    }
    var componentType: Int = MeshConstants.MESH_COMPONENT_UNKNOWN
    var deviceIsOn: Bool?
    var getDeviceOnOffTime: TimeInterval?
    static let GET_DEVICE_ONOFF_STATE_TIMEOUT: TimeInterval = 5.0      // unit: seconds
    
    var groupComponents: [GroupComponentData]?
    
    let pickerOverlayView = UIView(frame: CGRect(x: 0, y: 0, width: 50, height: 50))
    var isPickerOverlayViewMoving: Bool = false
    var topPoint: CGPoint?
    var leftPoint: CGPoint?
    var rightPoint: CGPoint?
    var bottomPoint: CGPoint?
    var centerPoint: CGPoint?
    var currentHSLPickerColor:UIColor?
    var currentCTLPickerColor:UIColor?
    var lastHSLPickerOverlayPosition: CGPoint?
    var lastCTLPickerOverlayPosition: CGPoint?
    var isLastTouchLiesInsidePickerControl: Bool = false
    
    var currentTemperatureValue: CGFloat?           // value range: LIGHT_TEMPERATURE_MIN ~ LIGHT_TEMPERATURE_MAX, unit: Kelvin.
    var currentHueValue: CGFloat?                   // value range: 0 ~ 360, uint: degree.
    var currentSaturationValue: CGFloat?            // value range: 0 ~ 100, unit: percentage.
    var currentLightnessValue: CGFloat? {           // value range: 1 ~ 100, unit: percentage.
        didSet {
            if let lightness = currentLightnessValue {
                self.brightnessSlider.value = Float(lightness)
                self.sliderPercentageLabel.text = String(format: "%.0f", lightness) + "%"
            }
        }
    }
    
    var pickerType: MeshControlPickerType = MeshControlPickerType.LIGHT_HSL
    let pickerTypeSuffixString: String = "  >"
    
    var trackingHelper: TrackingHelper = TrackingHelper()
    
    var isScrollEnabled: Bool = true
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        viewInit()
        notificationInit()
        defaultSettingInit()
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
        
        if isDeviceControl {
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
    }
    
    func viewInit() {
        self.nameLabel.text = self.deviceName ?? MeshConstantText.UNKNOWN_DEVICE_NAME
        
        if !self.isDeviceControl {
            self.nameView.isHidden = true
            self.nameViewHeightConstraint.constant = 0
        }
        
        // init picker overlay.
        pickerOverlayView.layer.cornerRadius = 25
        pickerOverlayView.layer.borderWidth = 2
        pickerOverlayView.layer.borderColor = UIColor.white.cgColor
        pickerView.addSubview(pickerOverlayView)
        
        // init peek and central points of picker image.
        if let imageView = self.pickerImage {
            let padding: CGFloat = 3  // there is no pixels at cornors, avoid move to that area.
            let imageWidth = imageView.frame.width
            let imageHeight = imageView.frame.height
            topPoint = CGPoint(x: imageWidth / 2, y: padding)
            leftPoint = CGPoint(x: padding, y: imageHeight / 2)
            rightPoint = CGPoint(x: imageWidth - padding, y: imageHeight / 2)
            bottomPoint = CGPoint(x: imageWidth / 2, y: imageHeight - padding)
            centerPoint = CGPoint(x: imageWidth / 2, y: imageHeight / 2)
        }
        
        scrollViewInit()
    }
    
    // disable view scroll if all subview within the whole screen.
    func scrollViewInit() {
        let rootBounds = self.view.bounds
        let bottomViewBounds = trunOffButton.convert(trunOffButton.bounds, to: self.view)
        if (bottomViewBounds.origin.y + bottomViewBounds.size.height) > (rootBounds.origin.y + rootBounds.size.height) {
            isScrollEnabled = true
        } else {
            isScrollEnabled = false
        }
        self.contentScrollView.isScrollEnabled = isScrollEnabled
    }
    
    func defaultSettingInit() {
        brightnessSlider.addTarget(self, action: #selector(brightnessSliderDidEndTouch), for: [.touchUpInside, .touchUpOutside])
        
        guard let deviceName = self.deviceName else {
            print("error: GroupDetailsControlsViewController, defaultSettingInit, invalid devicename is nil")
            return
        }
        
        // Get component type. For mesh group, the component type always be MeshConstants.MESH_COMPONENT_UNKNOWN.
        if self.componentType == MeshConstants.MESH_COMPONENT_UNKNOWN {
            self.componentType = MeshFrameworkManager.shared.getMeshComponentType(componentName: deviceName)
        }
        // Set the default picker image type and UI based on the mesh component type.
        self.setControlType(type: MeshControlPickerType.convertMeshComponentTypeToPickerType(componentType: self.componentType))
        
        // only the component device will monitor the status value update.
        if isDeviceControl {
            deviceIsOn = nil
            let lightTypes = [MeshControlPickerType.LIGHT_HSL, MeshControlPickerType.LIGHT_CTL, MeshControlPickerType.LIGHT_DIMMABLE]
            if lightTypes.contains(pickerType) {
                MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error: Int) in
                    guard error == MeshErrorCode.MESH_SUCCESS else {
                        /* wait for a little time to for connection completion. */
                        DispatchQueue.main.asyncAfter(deadline: DispatchTime.now() + .milliseconds(5000)) {
                            MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error: Int) in
                                guard error == MeshErrorCode.MESH_SUCCESS else {
                                    print("error: GroupDetailsControlsViewController, defaultSettingInit, failed to connect to mesh netowrk, error=\(error)")
                                    return
                                }
                                
                                self.getDeviceOnOffTime = Date().timeIntervalSince1970
                                let error = MeshFrameworkManager.shared.meshClientOnOffGet(deviceName: deviceName)
                                print("GroupDetailsControlsViewController, defaultSettingInit, meshClientOnOffGet, error=\(error)")
                            }
                        }
                        return
                    }
                    
                    self.getDeviceOnOffTime = Date().timeIntervalSince1970
                    let error = MeshFrameworkManager.shared.meshClientOnOffGet(deviceName: deviceName)
                    print("GroupDetailsControlsViewController, defaultSettingInit, meshClientOnOffGet, error=\(error)")
                }
            }
        }
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
                NetworkManager.shared.uploadMeshFiles { (error) in
                    print("GroupDetailsControlsViewController, MESH_NETWORK_DATABASE_CHANGED, uploadMeshFiles, error=\(error)")
                }
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_ON_OFF_STATUS):
            if let onOffStatus = MeshNotificationConstants.getOnOffStatus(userInfo: userInfo) {
                print("GroupDetailsControlsViewController, onOffStatusNotificationHandler, onOffStatus=\(onOffStatus)")
                if onOffStatus.deviceName == self.deviceName {
                    deviceIsOn = onOffStatus.isOn
                    // TODO[optional]: change the device icon images if required.
                    if let isOn = deviceIsOn {
                        let devceImageIconName = isOn ? MeshAppImageNames.brightnessOnImage : MeshAppImageNames.brightnessOffImage
                        deviceImageIcon.image = UIImage(named: devceImageIconName)
                    }
                    
                    // send GET messsage to remote device to sync the latest status if possible.
                    switch pickerType {
                    case .LIGHT_HSL:
                        let error = MeshFrameworkManager.shared.meshClientHslGet(deviceName: onOffStatus.deviceName)
                        print("GroupDetailsControlsViewController, onOffStatusNotificationHandler, meshClientHslGet, error=\(error)")
                    case .LIGHT_CTL:
                        let error = MeshFrameworkManager.shared.meshClientCtlGet(deviceName: onOffStatus.deviceName)
                        print("GroupDetailsControlsViewController, onOffStatusNotificationHandler, meshClientCtlGet, error=\(error)")
                    case .LIGHT_DIMMABLE:
                        let error = MeshFrameworkManager.shared.meshClientLightnessGet(deviceName: onOffStatus.deviceName)
                        print("GroupDetailsControlsViewController, onOffStatusNotificationHandler, meshClientLightnessGet, error=\(error)")
                    default:
                        break
                    }
                    
                    // save status value to UserSettings.
                    UserSettings.shared.setComponentStatus(componentName: onOffStatus.deviceName,
                                                           values: [MeshComponentValueKeys.onoff: onOffStatus.isOn])
                }
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_LEVEL_STATUS):
            if let levelStatus = MeshNotificationConstants.getLevelStatus(userInfo: userInfo) {
                print("GroupDetailsControlsViewController, levelStatusNotificationHandler, levelStatus=\(levelStatus)")
                
                if levelStatus.deviceName == self.deviceName {
                    deviceIsOn = (levelStatus.level == 0) ? false : true
                    // TODO[optional]: change the device icon images if required.
                    if let isOn = deviceIsOn {
                        let devceImageIconName = isOn ? MeshAppImageNames.brightnessOnImage : MeshAppImageNames.brightnessOffImage
                        deviceImageIcon.image = UIImage(named: devceImageIconName)
                    }
                    
                    UserSettings.shared.setComponentStatus(componentName: levelStatus.deviceName,
                                                           values: [MeshComponentValueKeys.level: levelStatus.level])
                }
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_HSL_STATUS):
            if let hslStatus = MeshNotificationConstants.getHslStatus(userInfo: userInfo) {
                print("GroupDetailsControlsViewController, hslStatusNotificationHandler, hslStatus=\(hslStatus)")
                
                if hslStatus.deviceName == self.deviceName {
                    deviceIsOn = (hslStatus.lightness == 0) ? false : true
                    // TODO[optional]: change the device icon images if required.
                    if let isOn = deviceIsOn {
                        let devceImageIconName = isOn ? MeshAppImageNames.brightnessOnImage : MeshAppImageNames.brightnessOffImage
                        deviceImageIcon.image = UIImage(named: devceImageIconName)
                    }
                    
                    self.currentLightnessValue = CGFloat(UtilityManager.meshRawValueToLighLightness(value: hslStatus.lightness))
                    self.currentHueValue = CGFloat(UtilityManager.meshRawValueToLightHue(value: hslStatus.hue))
                    self.currentSaturationValue = CGFloat(UtilityManager.meshRawValueToLightSaturation(value: hslStatus.saturation))
                    
                    UserSettings.shared.setComponentStatus(componentName: hslStatus.deviceName,
                                                           values: [MeshComponentValueKeys.hslLightness: self.currentLightnessValue!,
                                                                    MeshComponentValueKeys.hue: self.currentHueValue!,
                                                                    MeshComponentValueKeys.saturation: self.currentSaturationValue!])
                }
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_CTL_STATUS):
            if let ctlStatus = MeshNotificationConstants.getCtlStatus(userInfo: userInfo) {
                print("GroupDetailsControlsViewController, ctlStatusNotificationHandler, ctlStatus=\(ctlStatus)")
                
                if ctlStatus.deviceName == self.deviceName {
                    deviceIsOn = (ctlStatus.targetLightness == 0) ? false : true
                    // TODO[optional]: change the device icon images if required.
                    if let isOn = deviceIsOn {
                        let devceImageIconName = isOn ? MeshAppImageNames.brightnessOnImage : MeshAppImageNames.brightnessOffImage
                        deviceImageIcon.image = UIImage(named: devceImageIconName)
                    }
                    
                    self.currentLightnessValue = CGFloat(UtilityManager.meshRawValueToLighLightness(value: ctlStatus.targetLightness))
                    self.currentTemperatureValue = CGFloat(UtilityManager.meshRawValueToLightTemperature(value: ctlStatus.targetTemperature))
                    
                    UserSettings.shared.setComponentStatus(componentName: ctlStatus.deviceName,
                                                           values: [MeshComponentValueKeys.ctlLightness: self.currentLightnessValue!,
                                                                    MeshComponentValueKeys.temperature: self.currentTemperatureValue!])
                }
            }
        case Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_LIGHTNESS_STATUS):
            if let lightnessStatus = MeshNotificationConstants.getLightnessStatus(userInfo: userInfo) {
                print("GroupDetailsControlsViewController, lightnessStatusNotificationHandler, lightnessStatus=\(lightnessStatus)")
                
                if lightnessStatus.deviceName == self.deviceName {
                    deviceIsOn = (lightnessStatus.targetLightness == 0) ? false : true
                    // TODO[optional]: change the device icon images if required.
                    if let isOn = deviceIsOn {
                        let devceImageIconName = isOn ? MeshAppImageNames.brightnessOnImage : MeshAppImageNames.brightnessOffImage
                        deviceImageIcon.image = UIImage(named: devceImageIconName)
                    }
                    
                    self.currentLightnessValue = CGFloat(UtilityManager.meshRawValueToLighLightness(value: lightnessStatus.targetLightness))
                    
                    UserSettings.shared.setComponentStatus(componentName: lightnessStatus.deviceName,
                                                           values: [MeshComponentValueKeys.dimmableLightness: self.currentLightnessValue!])
                }
            }
        default:
            break
        }
    }
    
    func setControlType(type: MeshControlPickerType) {
        guard let deviceName = self.deviceName else { return }
        let typeButtonTitle = type.rawValue + pickerTypeSuffixString
        TrackingHelper.shared.stopTracking()
        self.pickerType = type
        self.controlTypeButton.setTitle(typeButtonTitle, for: .normal)
        self.controlTypeButton.setTitle(typeButtonTitle, for: .selected)
        
        // Try to fetch the last values for the mesh component. It may be a device or a group. If not existing, set to default value.
        let values = UserSettings.shared.getComponentStatus(componentName: deviceName) ?? [:]
        self.currentHueValue = values[MeshComponentValueKeys.hue] as? CGFloat ?? 0.0
        self.currentSaturationValue = values[MeshComponentValueKeys.saturation] as? CGFloat ?? 0.0
        self.currentTemperatureValue = values[MeshComponentValueKeys.temperature] as? CGFloat ?? 0.0
        switch type {
        case .LIGHT_CTL:
            self.pickerImage.image = UIImage(named: MeshAppImageNames.ctlColorImage)
            self.currentLightnessValue = values[MeshComponentValueKeys.ctlLightness] as? CGFloat ?? MeshAppConstants.LIGHT_MIN_VISIBLE_LIGHTNESS
        case .LIGHT_DIMMABLE:
            self.pickerImage.image = UIImage(named: MeshAppImageNames.cypressLogoImage)
            self.currentLightnessValue = values[MeshComponentValueKeys.dimmableLightness] as? CGFloat ?? MeshAppConstants.LIGHT_MIN_VISIBLE_LIGHTNESS
        default:
            self.currentLightnessValue = values[MeshComponentValueKeys.hslLightness] as? CGFloat ?? MeshAppConstants.LIGHT_MIN_VISIBLE_LIGHTNESS
            self.pickerImage.image = UIImage(named: MeshAppImageNames.hslColorImage)
        }
        self.currentLightnessValue = (self.currentLightnessValue! < MeshAppConstants.LIGHT_MIN_VISIBLE_LIGHTNESS) ? MeshAppConstants.LIGHT_MIN_VISIBLE_LIGHTNESS : self.currentLightnessValue
        
        updatePickerOverlayView()
    }
    
    func updatePickerOverlayView() {
        switch self.pickerType {
        case .LIGHT_HSL:
            if let lastPosition = self.lastHSLPickerOverlayPosition, let lastColor = self.currentHSLPickerColor {
                self.pickerOverlayView.layer.position = lastPosition
                self.pickerOverlayView.backgroundColor = lastColor
                return
            }
        case .LIGHT_CTL:
            if let lastPosition = self.lastCTLPickerOverlayPosition, let lastColor = self.currentCTLPickerColor {
                self.pickerOverlayView.layer.position = lastPosition
                self.pickerOverlayView.backgroundColor = lastColor
                return
            }
        default:
            break
        }
        
        // reset to default position and transparent color.
        self.pickerOverlayView.layer.position = CGPoint.zero
        self.pickerOverlayView.backgroundColor = UIColor.clear
    }
    
    @IBAction func onControlTypeButtonClick(_ sender: UIButton) {
        print("GroupDetailsControlsViewController, onControlTypeButtonClick")
        let choices = MeshControlPickerType.allValues
        let controller = PopoverChoiceTableViewController(choices: choices) { (index: Int, selection: String) in
            print("onControlTypeButtonClick, index=\(index), selection=\(selection)")
            guard let choice = MeshControlPickerType.init(rawValue: selection) else { return }
            self.setControlType(type: choice)
        }
        controller.preferredContentSize = CGSize(width: 120, height: controller.getPreferredPopoverViewSize())
        controller.showPopoverPresentation(parent: self, sourceView: sender as UIView, position: .right)
    }
    
    @IBAction func onBrightnessValueChanged(_ sender: UISlider) {
        var lightness = CGFloat(sender.value)
        lightness = (lightness < MeshAppConstants.LIGHT_MIN_VISIBLE_LIGHTNESS) ? MeshAppConstants.LIGHT_MIN_VISIBLE_LIGHTNESS : lightness
        self.currentLightnessValue = lightness.rounded()
        self.sliderPercentageLabel.text = String(format: "%.0f", lightness) + "%"
        print("GroupDetailsControlsViewController, onBrightnessValueChanged, value:\(sender.value), lightness=\(lightness)")
        
        if !TrackingHelper.shared.isTracking {
            TrackingHelper.shared.startTracking(componentType: MeshControlPickerType.convertPickerTypeToMeshComponentType(pickerType: pickerType))
        }
        trackingDataUpdate()
    }
    
    @IBAction func onTurnOnButtonClick(_ sender: UIButton) {
        print("GroupDetailsControlsViewController, onTurnOnButtonClick, deviceName:\(String(describing: self.deviceName)), isDeviceControl:\(self.isDeviceControl)")
        turnDeviceOnOff(isOn: true)
    }
    
    @IBAction func onTurnOffButtonClick(_ sender: UIButton) {
        print("GroupDetailsControlsViewController, onTurnOffButtonClick, deviceName:\(String(describing: self.deviceName)), isDeviceControl:\(self.isDeviceControl)")
        turnDeviceOnOff(isOn: false)
    }
    
    @objc func brightnessSliderDidEndTouch() {
        print("GroupDetailsControlsViewController, brightnessSliderDidEndTouch")
        // TODO: for test purpose, some device may do not support get status command.
        if false, isDeviceControl, deviceIsOn == nil, let startTime = getDeviceOnOffTime,
            (Date().timeIntervalSince1970 - startTime) > GroupDetailsControlsViewController.GET_DEVICE_ONOFF_STATE_TIMEOUT {
            UtilityManager.showAlertDialogue(parentVC: self,
                                             message: "The remote device may be not reachable or has been turned OFF. Please move it in range or turn it ON, then retry again.",
                                             title: "Warning")
            return
        }
        
        // Stop tracking data changes.
        // delay 500ms to avoid the last message was lost because of sent too much messages.
        self.onBrightnessValueChanged(self.brightnessSlider)
        DispatchQueue.main.asyncAfter(deadline: DispatchTime.now() + .milliseconds(500)) {
            TrackingHelper.shared.stopTracking()
        }
    }
    
    func trackingDataUpdate() {
        if let deviceName = self.deviceName {
            MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error: Int) in
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("error: GroupDetailsControlsViewController, trackingDataUpdate, failed to connect to mesh network, error=\(error)")
                    return
                }
                
                print("GroupDetailsControlsViewController, trackingDataUpdate, pickerType=\(self.pickerType.rawValue)")
                switch self.pickerType {
                case .LIGHT_HSL:
                    if let hue = self.currentHueValue, let satuation = self.currentSaturationValue, let lightness = self.currentLightnessValue {
                        let error = MeshFrameworkManager.shared.meshClientHslSet(deviceName: deviceName,
                                                                                 lightness: UtilityManager.lightLightnessToMeshRawValue(lightness: Double(lightness)),
                                                                                 hue: UtilityManager.lightHueToMeshRawValue(hue: Double(hue)),
                                                                                 saturation: UtilityManager.lightSaturationToMeshRawValue(saturation: Double(satuation)),
                                                                                 reliable: true,
                                                                                 transitionTime: UInt32(MeshAppConstants.LIGHT_HSL_TRANSITION_TIME),
                                                                                 delay: MeshAppConstants.LIGHT_HSL_DELAY_TIME)
                        print("GroupDetailsControlsViewController, trackingDataUpdate, meshClientHslSet, error=\(error)")
                        
                        // store the latest setting into UserSetting.
                        if error == MeshErrorCode.MESH_SUCCESS {
                            let values: [String: Any] = [MeshComponentValueKeys.hslLightness: lightness,
                                                         MeshComponentValueKeys.hue: hue,
                                                         MeshComponentValueKeys.saturation: satuation]
                            UserSettings.shared.setComponentStatus(componentName: deviceName, values: values)
                        }
                    }
                case .LIGHT_CTL:
                    if let lightness = self.currentLightnessValue, let temperature = self.currentTemperatureValue {
                        let error = MeshFrameworkManager.shared.meshClientCtlSet(deviceName: deviceName,
                                                                                 lightness: UtilityManager.lightLightnessToMeshRawValue(lightness: Double(lightness)),
                                                                                 temperature: UtilityManager.lightTemperatureToMeshRawValue(temperature: Double(temperature)),
                                                                                 deltaUv: 0,
                                                                                 reliable: true,
                                                                                 transitionTime: UInt32(MeshAppConstants.LIGHT_CTL_TRANSITION_TIME),
                                                                                 delay: MeshAppConstants.LIGHT_CTL_DELAY_TIME)
                        print("GroupDetailsControlsViewController, trackingDataUpdate, meshClientCtlSet, error=\(error)")
                        
                        // store the latest setting into UserSetting.
                        if error == MeshErrorCode.MESH_SUCCESS {
                            let values: [String: Any] = [MeshComponentValueKeys.ctlLightness: lightness,
                                                         MeshComponentValueKeys.temperature: temperature]
                            UserSettings.shared.setComponentStatus(componentName: deviceName, values: values)
                        }
                    }
                case .LIGHT_DIMMABLE:
                    if let lightness = self.currentLightnessValue {
                        let error = MeshFrameworkManager.shared.meshClientLightnessSet(deviceName: deviceName,
                                                                                       lightness: UtilityManager.lightLightnessToMeshRawValue(lightness: Double(lightness)),
                                                                                       reliable: true,
                                                                                       transitionTime: UInt32(MeshAppConstants.LIGHT_CTL_TRANSITION_TIME),
                                                                                       delay: MeshAppConstants.LIGHT_CTL_DELAY_TIME)
                        print("GroupDetailsControlsViewController, trackingDataUpdate, meshClientLightnessSet, error=\(error)")
                        
                        // store the latest setting into UserSetting.
                        if error == MeshErrorCode.MESH_SUCCESS {
                            let values: [String: Any] = [MeshComponentValueKeys.dimmableLightness: lightness]
                            UserSettings.shared.setComponentStatus(componentName: deviceName, values: values)
                        }
                    }
                default:
                    break
                }
            }
        }
    }
    
    func turnDeviceOnOff(isOn: Bool, reliable: Bool = true) {
        if let deviceName = self.deviceName {
            MeshFrameworkManager.shared.runHandlerWithMeshNetworkConnected { (error) in
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("error: GroupDetailsControlsViewController, turnDeviceOnOff(deviceName:\(deviceName), isOn:\(isOn)), failed to connect to the mesh network")
                    return
                }
                
                let error = MeshFrameworkManager.shared.meshClientOnOffSet(deviceName: deviceName, isOn: isOn, reliable: reliable)
                guard error == MeshErrorCode.MESH_SUCCESS else {
                    print("error: GroupDetailsControlsViewController, meshClientOnOffSet(deviceName:\(deviceName), isOn:\(isOn), reliable=\(reliable)) failed, error=\(error)")
                    return
                }
                print("GroupDetailsControlsViewController, meshClientOnOffSet(deviceName:\(deviceName), isOn:\(isOn), reliable=\(reliable)) message sent out success")
                
                // if not reliable, simulating to send notification data to update the brightness image.
                if self.isDeviceControl, !reliable {
                    let notification = Notification.init(name: Notification.Name(rawValue: MeshNotificationConstants.MESH_CLIENT_ON_OFF_STATUS),
                                                           object: nil,
                                                           userInfo: [MeshNotificationConstants.USER_INFO_KEY_DEVICE_NAME: deviceName,
                                                                      MeshNotificationConstants.USER_INFO_KEY_TARGET: (isOn ? 1 : 0),
                                                                      MeshNotificationConstants.USER_INFO_KEY_PRESENT: (isOn ? 0 : 1),
                                                                      MeshNotificationConstants.USER_INFO_KEY_REMAINING_TIME: 0])
                    self.notificationHandler(notification)
                }
                
                // store the latest setting into UserSetting.
                let values: [String: Any] = [MeshComponentValueKeys.onoff: isOn]
                UserSettings.shared.setComponentStatus(componentName: deviceName, values: values)
            }
        }
    }
}

extension GroupDetailsControlsViewController {
    func touchInColorPickerArea(touch: UITouch, event: UIEvent?) -> Bool {
        let touchPoint = touch.location(in: self.pickerImage)
        if touchPoint.x < 0 || touchPoint.x > self.pickerImage.frame.width ||
            touchPoint.y < 0 || touchPoint.y > self.pickerImage.frame.height {
            return false
        }
        return true
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard (pickerType == .LIGHT_CTL || pickerType == .LIGHT_HSL) else { return }
        // TODO: for test purpose, some device may do not support get status command.
        if false, isDeviceControl, deviceIsOn == nil, let startTime = getDeviceOnOffTime,
            (Date().timeIntervalSince1970 - startTime) > GroupDetailsControlsViewController.GET_DEVICE_ONOFF_STATE_TIMEOUT {
            UtilityManager.showAlertDialogue(parentVC: self,
                                             message: "The remote device may be not reachable or has been turned OFF. Please move it in range or turn it ON, then retry again.",
                                             title: "Warning")
            return
        }
        
        if let touch = touches.first {
            // when touch in color picker area, stop scroll to make it easier picker color.
            if touchInColorPickerArea(touch: touch, event: event) {
                self.contentScrollView.isScrollEnabled = false
            }
            
            // User touched UI to change the data, start tracking data changes.
            TrackingHelper.shared.startTracking(componentType: MeshControlPickerType.convertPickerTypeToMeshComponentType(pickerType: self.pickerType))
            
            isPickerOverlayViewMoving = false
            touchProcess(at: touch.location(in: self.pickerView))
        }
    }
    
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard (pickerType == .LIGHT_CTL || pickerType == .LIGHT_HSL) else { return }
        
        if let touch = touches.first {
            if touchInColorPickerArea(touch: touch, event: event) && self.contentScrollView.isScrollEnabled {
                self.contentScrollView.isScrollEnabled = false
            }
            
            isPickerOverlayViewMoving = true
            touchProcess(at: touch.location(in: self.pickerView))
            
            // update tracking data.
            trackingDataUpdate()
        }
    }
    
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard (pickerType == .LIGHT_CTL || pickerType == .LIGHT_HSL) else { return }
        
        if let touch = touches.first {
            isPickerOverlayViewMoving = false
            touchProcess(at: touch.location(in: self.pickerView))
        }
        // Reenable scroll after touch ended.
        if isScrollEnabled {
            self.contentScrollView.isScrollEnabled = true
        }
        
        // Stop tracking data changes.
        // delay 500ms to avoid the last message was lost because of sent too much messages.
        self.trackingDataUpdate()
        DispatchQueue.main.asyncAfter(deadline: DispatchTime.now() + .milliseconds(500)) {
            TrackingHelper.shared.stopTracking()
        }
    }
    
    open override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        self.touchesEnded(touches, with: event)
    }
    
    func touchProcess(at touchPoint: CGPoint) {
        if let topPoint = self.topPoint, let leftPoint = self.leftPoint,
            let rightPoint = self.rightPoint, let bottomPoint = self.bottomPoint, let centerPoint = self.centerPoint {
            let quadrant = UtilityManager.getQuadrant(for: touchPoint,
                                                      bound: (topPoint: topPoint, leftPoint: leftPoint,
                                                              rightPoint: rightPoint, bottomPoint: bottomPoint))
            var tp1 = CGPoint.zero
            var tp2 = CGPoint.zero
            switch quadrant {
            case .TopLeft:
                tp1 = topPoint
                tp2 = leftPoint
            case .TopRight:
                tp1 = topPoint
                tp2 = rightPoint
            case .BottomLeft:
                tp1 = bottomPoint
                tp2 = leftPoint
            case .BottomRight:
                tp1 = bottomPoint
                tp2 = rightPoint
            default:
                return
            }
            
            let intersectionPoint = UtilityManager.getIntersectionPoint(for: (p1: tp1, p2: tp2),
                                                                        and: (p1: touchPoint, p2: centerPoint))
            let isInsideColorPicker = UtilityManager.pointLiesInsideTringle(point: touchPoint, tp1: tp1, tp2: tp2, tp3: centerPoint)
            if isInsideColorPicker {
                calculateValuesInsidePickerBoundary(with: touchPoint, intersectionPoint: intersectionPoint)
            } else {
                calculateValuesOutsidePickerBoundary(with: touchPoint, intersectionPoint: intersectionPoint)
            }
        }
    }
    
    func calculateValuesInsidePickerBoundary(with touchPoint: CGPoint, intersectionPoint: CGPoint) {
        self.isLastTouchLiesInsidePickerControl = true
        
        setPickerOverlayColor(at: touchPoint)
        switch self.pickerType {
        case .LIGHT_HSL:
            calculateHueValue(at: touchPoint)
            calculateColorSaturationPercentage(at: touchPoint, intersectionPoint: intersectionPoint, isInsidePickerBoundary: true)
        case .LIGHT_CTL:
            calculateCTLTempreture(at: touchPoint)
        default:
            break
        }
    }
    
    func calculateValuesOutsidePickerBoundary(with touchPoint: CGPoint, intersectionPoint: CGPoint) {
        if !self.isPickerOverlayViewMoving {
            self.isLastTouchLiesInsidePickerControl = false
        }
        
        if self.isPickerOverlayViewMoving && self.isLastTouchLiesInsidePickerControl {
            setPickerOverlayColor(at: intersectionPoint)
            switch self.pickerType {
            case .LIGHT_HSL:
                calculateColorSaturationPercentage(at: touchPoint, intersectionPoint: intersectionPoint, isInsidePickerBoundary: false)
            case .LIGHT_CTL:
                calculateCTLTempreture(at: intersectionPoint)
            default:
                break
            }
        }
    }
    
    func setPickerOverlayColor(at position: CGPoint) {
        if let pixelColor = self.pickerImage?.image?.getPixelColor(at: position, in: pickerImage.frame) {
            switch self.pickerType {
            case .LIGHT_HSL:
                self.currentHSLPickerColor = pixelColor
                self.lastHSLPickerOverlayPosition = position
            case .LIGHT_CTL:
                self.currentCTLPickerColor = pixelColor
                self.lastCTLPickerOverlayPosition = position
            default:
                break
            }
        }
        
        updatePickerOverlayView()
    }
    
    func calculateHueValue(at position: CGPoint) {
        if let hslColor = self.currentHSLPickerColor {
            var hue: CGFloat = 0
            var saturation: CGFloat = 0
            var brighness: CGFloat = 0
            var alpha: CGFloat = 0
            /*
             * The return value range of UIColor.getHue() is 0.0 ~ 1.0. but the HSL color space the value range is 0.0 ~ 360.0 degree,
             * so, the converted value should be remapped to HSL color space value range.
             */
            if hslColor.getHue(&hue, saturation: &saturation, brightness: &brighness, alpha: &alpha) {
                self.currentHueValue = hue * 360
            }
        }
    }
    
    func calculateColorSaturationPercentage(at touchPoint: CGPoint, intersectionPoint: CGPoint, isInsidePickerBoundary: Bool) {
        // calculate saturation percentage value based on the touch point distance to the center point.
        var saturationPercentage = CGFloat(MeshConstants.LIGHT_SATURATION_PERCENTAGE_MAX)
        if isInsidePickerBoundary, let centerPoint = self.centerPoint {
            let maxDistance = UtilityManager.getDistance(between: intersectionPoint, and: centerPoint)
            let distance = UtilityManager.getDistance(between: touchPoint, and: centerPoint)
            saturationPercentage = (distance / maxDistance * CGFloat(MeshConstants.LIGHT_SATURATION_PERCENTAGE_MAX)).rounded()
        }
        self.currentSaturationValue = saturationPercentage
        print("calculateColorSaturationPercentage, saturationPercentage=\(saturationPercentage) %")
    }
    
    func calculateCTLTempreture(at position: CGPoint) {
        if let topPoint = self.topPoint, let bottomPoint = self.bottomPoint, position.y >= topPoint.y, position.y <= bottomPoint.y {
            // Calculate range * percentage + min
            let maxYPoints: CGFloat = bottomPoint.y - topPoint.y
            let positionYPoints: CGFloat = position.y - topPoint.y
            let deviceTemperatureRange: CGFloat = MeshAppConstants.LIGHT_TEMPERATURE_MAX - MeshAppConstants.LIGHT_TEMPERATURE_MIN
            let temperature: CGFloat = MeshAppConstants.LIGHT_TEMPERATURE_MIN + deviceTemperatureRange * positionYPoints / maxYPoints
            self.currentTemperatureValue = temperature.rounded()

            print("calculateCTLTempreture, temperature:\(String(describing: self.currentTemperatureValue)), LIGHT_TEMPERATURE_MIN:\(MeshAppConstants.LIGHT_TEMPERATURE_MIN)")
            print("calculateCTLTempreture, positionYPoints:\(positionYPoints), maxYPoints:\(maxYPoints)")
        }
    }
    
    
    
}

extension UIImage {
    func getPixelColor(at location: CGPoint, in frame: CGRect) -> UIColor {
        guard let pixelData = self.cgImage?.dataProvider?.data else {
            print("error: GroupDetailsControlsViewController, getPixelColor is nil")
            return UIColor.white
        }
        let x: CGFloat = self.size.width * location.x / frame.width
        let y: CGFloat = self.size.height * location.y / frame.height
        let pixelPoint = CGPoint(x: x, y: y)
        let data: UnsafePointer<UInt8> = CFDataGetBytePtr(pixelData)
        let pixelIndex: Int = ((Int(self.size.width) * Int(pixelPoint.y)) + Int(pixelPoint.x)) * 4
        let r = CGFloat(data[pixelIndex]) / CGFloat(255.0)
        let g = CGFloat(data[pixelIndex+1]) / CGFloat(255.0)
        let b = CGFloat(data[pixelIndex+2]) / CGFloat(255.0)
        let a = CGFloat(data[pixelIndex+3]) / CGFloat(255.0)
        return UIColor(red: r, green: g, blue: b, alpha: a)
    }
}

// Make sure the touch event to be transfer down to this VC.
extension UIScrollView {
    open override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesBegan(touches, with: event)
    }
    
    open override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesMoved(touches, with: event)
    }
    
    open override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesEnded(touches, with: event)
    }
    
    open override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesCancelled(touches, with: event)
    }
}
