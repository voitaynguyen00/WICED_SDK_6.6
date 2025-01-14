/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * This file implements the MeshDevice class which stands for a mesh device in a mesh network in the App.
 */

import Foundation
import CoreBluetooth

public class MeshDevice: NSObject, NSCoding {
    
    public let uuid: UUID           // UUID uniquely identifies a mesh device, must be existing and won't change during mesh device's whole lift time.
    public var name: String? {      // Provisioned mesh device name, it could be changed after device provisioned, so use calculate value.
        return MeshFrameworkManager.shared.getMeshComponentsByDevice(uuid: uuid)?.first
    }
    public var type: Int {          // mesh device type, see MeshConstants.MESH_COMPONENT_* for detail list of mesh device type.
        if let meshName = name {
            return MeshFrameworkManager.shared.getMeshComponentType(componentName: meshName)
        } else {
            return MeshConstants.MESH_COMPONENT_UNKNOWN
        }
    }
    public var unprovisionedName: String?   // The device name before be provisioned into mesh network. Such as the name seen in the advert data.
    
    public init(uuid: UUID) {
        self.uuid = uuid
        super.init()
    }
    
    public func encode(with aCoder: NSCoder) {
        aCoder.encode(self.uuid, forKey: "uuid")
        aCoder.encode(self.unprovisionedName, forKey: "unprovisionedName")
    }
    
    required public init?(coder aDecoder: NSCoder) {
        self.uuid = aDecoder.decodeObject(forKey: "uuid") as! UUID
        self.unprovisionedName = aDecoder.decodeObject(forKey: "unprovisionedName") as? String
        super.init()
    }
}

public class MeshDeviceManager: NSObject {
    public static let shared = MeshDeviceManager()
    private var mMeshDevices: [MeshDevice] = []
    
    public override init() {
        super.init()
        mMeshDevices = MeshFrameworkManager.shared.provisionedMeshDevices
    }
    
    public var meshDevices: [MeshDevice] {
        return mMeshDevices
    }
    
    public func addMeshDevice(device: MeshDevice) {
        if let _ = mMeshDevices.filter({$0.uuid == device.uuid}).first {
            return  // the mesh device has already been added.
        }
        mMeshDevices.append(device)
        MeshFrameworkManager.shared.provisionedMeshDevices = mMeshDevices
    }
    
    public func addMeshDevice(by uuid: UUID) {
        addMeshDevice(device: MeshDevice(uuid: uuid))
    }
    
    public func removeMeshDevice(by uuid: UUID) {
        for index in 0..<mMeshDevices.count {
            if mMeshDevices[index].uuid == uuid {
                mMeshDevices.remove(at: index)
                MeshFrameworkManager.shared.provisionedMeshDevices = mMeshDevices
                return
            }
        }
    }
    
    public func removeMeshDevice(by name: String) {
        var removeIndex: [Int] = []
        for index in 0..<mMeshDevices.count {
            if mMeshDevices[index].name == name {
                removeIndex.append(index)
            }
        }
        for index in removeIndex {
            mMeshDevices.remove(at: index)
        }
    }
    
    public func removeMeshDevice(at index: Int) {
        guard index < mMeshDevices.count, index >= 0 else { return }
        mMeshDevices.remove(at: index)
    }
    
    public func clearMeshDevices() {
        mMeshDevices.removeAll()
        MeshFrameworkManager.shared.provisionedMeshDevices = mMeshDevices
    }
    
    public func getMeshDevice(by uuid: UUID) -> MeshDevice? {
        return mMeshDevices.filter({$0.uuid == uuid}).first
    }
    
    public func getMeshDevice(by name: String) -> [MeshDevice] {
        return mMeshDevices.filter({$0.name == name})
    }
    
    public func getMeshDevice(at index: Int) -> MeshDevice? {
        guard index < mMeshDevices.count, index >= 0 else {
            return nil
        }
        return mMeshDevices[index]
    }
}


/*
public class MeshDevice: NSObject {
    public static let BLUETOOTH_RSSI_NOT_AVAIABLE: Int8 = 127
    public static let BLUETOOTH_RSSI_MAXIMUM_VALUE: Int8 = 20
    public static let BLUETOOTH_RSSI_MINIMUM_VALUE: Int8 = -127
    
    public static let DEFAULT_UNKNOWN_NAME = "Unknown Name"
    
    var peripheral: CBPeripheral?
    
    var gattService: CBService?
    var gattCharacteristic: CBCharacteristic?
    var gattNotifyCharacteristic: CBCharacteristic?
    
    var advertisementData: [String : Any]?
    public var rssi: Int8 = MeshDevice.BLUETOOTH_RSSI_NOT_AVAIABLE
    public var rssiString: String {
        if rssi == MeshDevice.BLUETOOTH_RSSI_NOT_AVAIABLE {
            return "unavaiable dBm"
        } else {
            return "\(rssi) dBm"
        }
    }
    
    public var isConnectable: Bool {
        if let advData = advertisementData, let connectable = advData[CBAdvertisementDataIsConnectable] as? Bool {
            return connectable
        }
        return false
    }
    
    public let uuid: UUID       // UUID uniquely identifies a mesh device.
    public var name: String?    // Provisioned mesh device name.
    
    public init(uuid: UUID) {
        self.uuid = uuid
        super.init()
    }
    
    public func aainit(peripheral: CBPeripheral, advertisementData: [String : Any]? = nil, rssi: NSNumber = NSNumber(value: MeshDevice.BLUETOOTH_RSSI_NOT_AVAIABLE)) {
        self.peripheral = peripheral
        self.advertisementData = advertisementData
        if rssi.int8Value >= MeshDevice.BLUETOOTH_RSSI_MINIMUM_VALUE, rssi.int8Value <= MeshDevice.BLUETOOTH_RSSI_MAXIMUM_VALUE {
            self.rssi = MeshDevice.BLUETOOTH_RSSI_NOT_AVAIABLE
        } else {
            self.rssi = rssi.int8Value
        }
    }
}
*/
