/*
 * Copyright Cypress Semiconductor
 */

/** @file
 *
 * This file implements the MeshStorageSettings class which manages the storages for reading and writing
 * mesh library automatically generated and managed .json and .bin files.
 */

import Foundation

class MeshStorageSettings {
    static let shared = MeshStorageSettings()
    private var meshStoragePath: String?
    private var userIdentify: String?
    private var userStorageName: String?
    private var userStoragePath: String?
    private var jsonFilePath: String?
    private var provisionerUuidBinFilePath: String?
    private var replayProtectionListBinFilePath: String?
    
    private var jsonFileName: String?
    private var provisionerUuidBinFileName: String?
    private var replayProtectionListBinFileName: String?
    
    private var defaultFwImagesStorageName: String = "fwImages"
    private var fwImagesStoragePath: String?
    
    var currentMeshStoragepath: String? {
        return meshStoragePath
    }
    
    var currentUserIdentify: String? {
        return userIdentify
    }
    
    var currentUserStorageName: String? {
        return userStorageName
    }
    
    var currentUserStoragePath: String? {
        return userStoragePath
    }
    
    var fwImagesPath: String? {
        return fwImagesStoragePath
    }
    
    var exportMeshStorageName: String {
        return "ExportedMeshNetworks"
    }
    
    var exportMeshStoragePath: String? {
        guard let rootStoragePath = currentMeshStoragepath else {
            return nil
        }
        return rootStoragePath + "/" + exportMeshStorageName
    }
    
    var currentAllFileNameUnderUserStorage: [String]? {
        guard let _ = currentUserStoragePath else {
            print("error: currentAllFileNameUnderUserStorage, invalid MeshStorageSettings, currentUserStoragePath is nil")
            return nil
        }
        
        var fileNames: [String] = []
        if let storagePath = currentUserStoragePath {
            if let contents = try? FileManager.default.contentsOfDirectory(atPath: storagePath) {
                for fileContent in contents {
                    if fileContent.hasSuffix(".json") || fileContent.hasSuffix(".bin") {
                        fileNames.append(fileContent)
                    }
                }
            }
        }
        return fileNames
    }
    
    var currentAllFilePathUnderUserStorage: [String]? {
        guard let fileStoragePath = currentUserStoragePath else {
            print("error: currentAllFilePathUnderUserStorage, invalid MeshStorageSettings, currentUserStoragePath is nil")
            return nil
        }
        
        var filePaths: [String] = []
        if let storagePath = currentUserStoragePath {
            if let contents = try? FileManager.default.contentsOfDirectory(atPath: storagePath) {
                for fileContent in contents {
                    if fileContent.hasSuffix(".json") || fileContent.hasSuffix(".bin") {
                        filePaths.append(fileStoragePath.appending("/").appending(fileContent))
                    }
                }
            }
        }
        return filePaths
    }
    
    var meshNetworksUnderExportedMeshStorage: [String]? {
        guard let exportedStoragePath = exportMeshStoragePath else {
            print("error: meshNetworksUnderExportedMeshStorage, invalid exportMeshStoragePath nil")
            return nil
        }

        var networks: [String] = []
        if let files = try? FileManager.default.contentsOfDirectory(atPath: exportedStoragePath) {
            for fileName in files {
                if fileName.hasSuffix(".json") {
                    let extIndex = fileName.lastIndex(of: ".") ?? fileName.endIndex
                    networks.append(String(fileName.prefix(upTo: extIndex)))
                }
            }
        }
        return networks
    }
    
    func writeExportedMeshNetwork(networkName: String, jsonContent: String) -> Int {
        guard let rootStoragePath = self.meshStoragePath else {
            print("error: writeExportedMeshNetwork, mesh root storage directory not initialized yet")
            return MeshErrorCode.MESH_ERROR_INVALID_ARGS
        }
        
        let exportStoragePath = rootStoragePath + "/" + exportMeshStorageName
        let fileManager = FileManager.default
        var isDirectory = ObjCBool(false)
        let exists = fileManager.fileExists(atPath: exportStoragePath, isDirectory: &isDirectory)
        if !exists {
            do {
                try fileManager.createDirectory(atPath: exportStoragePath, withIntermediateDirectories: true, attributes: nil)
                print("writeExportedMeshNetwork, success on create exported mesh networks storage directory \(rootStoragePath)")
            } catch {
                print("error: writeExportedMeshNetwork, failed to create exported mesh networks storage directory \(rootStoragePath), \(error)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_CREATE_FAILED
            }
        }
        
        if !fileManager.isReadableFile(atPath: exportStoragePath) || !fileManager.isWritableFile(atPath: exportStoragePath) {
            print("error: writeExportedMeshNetwork, readbale=\(fileManager.isReadableFile(atPath: exportStoragePath)) or writeable=\(fileManager.isWritableFile(atPath: exportStoragePath)) not allowed at path \(exportStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_RW_NOT_ALLOWED
        }
        
        let exportFileName = exportStoragePath + "/" + networkName + ".json"
        if FileManager.default.createFile(atPath: exportFileName, contents: jsonContent.data(using: .utf8)) {
            print("writeExportedMeshNetwork, write exported file to \"\(exportFileName)\" success")
            return MeshErrorCode.MESH_SUCCESS
        }
        print("writeExportedMeshNetwork, failed to write exported file to \"\(exportFileName)\"")
        return MeshErrorCode.MESH_ERROR_FILE_CREATE_FAILED
    }
    
    func readExportedMeshNetwork(networkName: String) -> String? {
        guard let exportMeshStoragePath = self.exportMeshStoragePath else {
            print("error: readExportedMeshNetwork, mesh exported root storage directory not initialized yet")
            return nil
        }
        
        let fileManager = FileManager.default
        let importFilePath = exportMeshStoragePath + "/" + networkName + ".json"
        var isDirectory = ObjCBool(false)
        let exists = fileManager.fileExists(atPath: importFilePath, isDirectory: &isDirectory)
        if !exists || isDirectory.boolValue {
            print("error: readExportedMeshNetwork, mesh network file: \(importFilePath) not exists")
            return nil
        }
        if let networkContent = FileManager.default.contents(atPath: importFilePath), let jsonContent = String(data: networkContent, encoding: .utf8) {
            print("readExportedMeshNetwork file: \(importFilePath) success")
            return jsonContent
        }
        
        print("error: readExportedMeshNetwork, failed to read network file: \(importFilePath)")
        return nil
    }
    
    func setMeshRootStorage(rootStoragePath: String, storageName: String = "mesh") -> Int {
        let fileManager = FileManager.default
        var isDirectory = ObjCBool(false)
        var exists = fileManager.fileExists(atPath: rootStoragePath, isDirectory: &isDirectory)
        if !exists {
            do {
                try fileManager.createDirectory(atPath: rootStoragePath, withIntermediateDirectories: true, attributes: nil)
                print("setMeshRootStorage, success on create root storage directory \(rootStoragePath)")
            } catch {
                print("error: setMeshRootStorage, failed to create root directory \(rootStoragePath), \(error)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_CREATE_FAILED
            }
        }

        if !fileManager.isReadableFile(atPath: rootStoragePath) || !fileManager.isWritableFile(atPath: rootStoragePath) {
            print("error: setMeshRootStorage, readbale=\(fileManager.isReadableFile(atPath: rootStoragePath)) or writeable=\(fileManager.isWritableFile(atPath: rootStoragePath)) not allowed at path \(rootStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_RW_NOT_ALLOWED
        }
        
        let lastCharacterIsSlash = (rootStoragePath.last == Character("/")) ? true : false
        let meshStoragePath = rootStoragePath + (lastCharacterIsSlash ? "\(storageName)" : "/\(storageName)")
        isDirectory = ObjCBool(false)
        exists = fileManager.fileExists(atPath: meshStoragePath, isDirectory: &isDirectory)
        if exists && isDirectory.boolValue {
            // The mesh root storage path has been existing.
            print("MeshStorageSettings, setMeshRootStorage, mesh root storage path exists")
        } else if exists && !isDirectory.boolValue {
            // A file with same name as @meshStoragePath has existed.
            print("error: setMeshRootStorage, file existed, unable to create directory \(meshStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_DUPLICATED_NAME
        } else {
            do {
                try fileManager.createDirectory(atPath: meshStoragePath, withIntermediateDirectories: true, attributes: nil)
            } catch {
                print("error: setMeshRootStorage, failed to create directory \(meshStoragePath), \(error)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_CREATE_FAILED
            }
        }
        
        if !fileManager.changeCurrentDirectoryPath(meshStoragePath) {
            print("error: setMeshRootStorage, failed to change current directory to \(meshStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_CHANGE_FAILED
        }
        
        self.meshStoragePath = fileManager.currentDirectoryPath
        print("MeshStorageSettings, setMeshRootStorage to \(String(describing: self.meshStoragePath)) success")
        
        _ = setMeshFwImagesStorage()
        return MeshErrorCode.MESH_SUCCESS
    }
    
    func setMeshFwImagesStorage(rootStoragePath: String? = nil, storageName: String = "fwImages") -> Int {
        var rootStorage = rootStoragePath
        if rootStorage == nil {
            rootStorage = self.meshStoragePath
        }
        guard let meshStorage = rootStorage else {
            print("error: setMeshFwImagesStorage, invalid root storage directory path")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_NOT_EXIST
        }

        let fileManager = FileManager.default
        var isDirectory = ObjCBool(false)
        var exists = fileManager.fileExists(atPath: meshStorage, isDirectory: &isDirectory)
        if !exists {
            do {
                try fileManager.createDirectory(atPath: meshStorage, withIntermediateDirectories: true, attributes: nil)
                print("setMeshFwImagesStorage, success on create mesh storage directory \(meshStorage)")
            } catch {
                print("error: setMeshFwImagesStorage, failed to create mesh directory \(meshStorage), \(error)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_CREATE_FAILED
            }
        }
        
        if !fileManager.isReadableFile(atPath: meshStorage) || !fileManager.isWritableFile(atPath: meshStorage) {
            print("error: setMeshFwImagesStorage, readbale=\(fileManager.isReadableFile(atPath: meshStorage)) or writeable=\(fileManager.isWritableFile(atPath: meshStorage)) not allowed at path \(meshStorage)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_RW_NOT_ALLOWED
        }
        
        let lastCharacterIsSlash = (meshStorage.last == Character("/")) ? true : false
        let fwImagesStoragePath = meshStorage + (lastCharacterIsSlash ? "\(storageName)" : "/\(storageName)")
        isDirectory = ObjCBool(false)
        exists = fileManager.fileExists(atPath: fwImagesStoragePath, isDirectory: &isDirectory)
        if exists && isDirectory.boolValue {
            // The mesh root storage path has been existing.
            print("setMeshFwImagesStorage, setMeshRootStorage, mesh root storage path exists")
        } else if exists && !isDirectory.boolValue {
            // A file with same name as @meshStoragePath has existed.
            print("error: setMeshFwImagesStorage, file existed, unable to create directory \(fwImagesStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_DUPLICATED_NAME
        } else {
            do {
                try fileManager.createDirectory(atPath: fwImagesStoragePath, withIntermediateDirectories: true, attributes: nil)
            } catch {
                print("error: setMeshFwImagesStorage, failed to create directory \(fwImagesStoragePath), \(error)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_CREATE_FAILED
            }
        }
        
        self.fwImagesStoragePath = fwImagesStoragePath
        print("MeshStorageSettings, setMeshFwImagesStorage to \(String(describing: self.fwImagesStoragePath)) success")
        return MeshErrorCode.MESH_SUCCESS
    }

    func setUserStorage(for user: String) -> Int {
        let fileManager = FileManager.default
        guard let meshStoragePath = self.meshStoragePath else {
            print("error: setUserStorage, setUserStorage, meshStoragePath not set")
            return MeshErrorCode.MESH_ERROR_INVALID_ARGS
        }
        if !fileManager.isReadableFile(atPath: meshStoragePath) || !fileManager.isWritableFile(atPath: meshStoragePath) {
            print("error: setUserStorage, readbale or writeable not allowed at path=\(meshStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_RW_NOT_ALLOWED
        }
        
        let userStorageName = MeshUtility.MD5(string: user)
        let userStoragePath = meshStoragePath + "/\(userStorageName)"
        var isDirectory = ObjCBool(false)
        let exists = fileManager.fileExists(atPath: userStoragePath, isDirectory: &isDirectory)
        if exists && isDirectory.boolValue {
            // The user storage path has been existing.
            print("MeshStorageSettings, setUserStorage, user storage path exists")
        } else if exists && !isDirectory.boolValue {
            // A file with same name as @userStoragePath has existed.
            print("error: setUserStorage, file existed, unable to create directory \(userStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_DUPLICATED_NAME
        } else {
            do {
                try fileManager.createDirectory(atPath: userStoragePath, withIntermediateDirectories: true, attributes: nil)
            } catch {
                print("error: setUserStorage, failed to create directory \(userStoragePath), \(error)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_CREATE_FAILED
            }
        }
        
        if !fileManager.changeCurrentDirectoryPath(userStoragePath) {
            print("error: setUserStorage, failed to change current directory to \(userStoragePath)")
            return MeshErrorCode.MESH_ERROR_DIRECTORY_CHANGE_FAILED
        }
        
        self.userIdentify = user
        self.userStorageName = userStorageName
        self.userStoragePath = fileManager.currentDirectoryPath
        print("MeshStorageSettings, setUserStorage forUser=\(user) under path=\(String(describing: self.userStoragePath)) success")
        return MeshErrorCode.MESH_SUCCESS
    }
    
    func deleteUserStorage(for user: String) -> Int{
        let fileManager = FileManager.default
        var isDirectory = ObjCBool(false)
        guard let meshStoragePath = self.meshStoragePath else {
            print("error: MeshStorageSettings, deleteUserStorage, invalid mesh storage status, meshStoragePath is nil")
            return MeshErrorCode.MESH_ERROR_INVALID_ARGS
        }
        
        let userStorageName = MeshUtility.MD5(string: user)
        let userStoragePath = meshStoragePath + "/\(userStorageName)"
        let exists = fileManager.fileExists(atPath: userStoragePath, isDirectory: &isDirectory)
        if exists {
            do {
                try fileManager.removeItem(atPath: userStoragePath)
            } catch {
                print("error: deleteUserStorage forUser=\(user), failed to delete directory \(userStoragePath)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_DELETE_FAILED
            }
        }
        
        print("MeshStorageSettings, deleteUserStorage forUser=\(user), \(userStoragePath) success")
        return MeshErrorCode.MESH_SUCCESS
    }
    
    func deleteMeshStorage() -> Int {
        let fileManager = FileManager.default
        var isDirectory = ObjCBool(false)
        guard let meshStoragePath = self.meshStoragePath else {
            print("error: MeshStorageSettings, deleteUserStorage, invalid mesh storage status, meshStoragePath is nil")
            return MeshErrorCode.MESH_ERROR_INVALID_ARGS
        }
        
        let exists = fileManager.fileExists(atPath: meshStoragePath, isDirectory: &isDirectory)
        if exists {
            do {
                try fileManager.removeItem(atPath: meshStoragePath)
            } catch {
                print("error: deleteMeshStorage, failed to delete directory \(meshStoragePath)")
                return MeshErrorCode.MESH_ERROR_DIRECTORY_DELETE_FAILED
            }
        }
        
        print("MeshStorageSettings, deleteMeshStorage, \(meshStoragePath) success")
        return MeshErrorCode.MESH_SUCCESS
    }
    
    func restoreMeshFile(fileName: String, content: Data?) -> Bool {
        if !validateMeshFilePath(fileName: fileName) {
            return false
        }
        return FileManager.default.createFile(atPath: fileName, contents: content)
    }
    
    func readMeshFile(fileName: String) -> Data? {
        if !validateMeshFilePath(fileName: fileName) {
            return nil
        }
        return FileManager.default.contents(atPath: fileName)
    }
    
    func validateMeshFilePath(fileName: String) -> Bool {
        guard let _ = userStoragePath else {
            print("error: MeshStorageSettings, validateMeshFilePath, \(fileName), not initialized mesh user storage path")
            return false
        }
        if !fileName.hasSuffix(".bin") && !fileName.hasSuffix(".json") {
            print("error: MeshStorageSettings, validateMeshFilePath, \(fileName), invalid mesh file extension")
            return false
        }
        
        var isDirectory = ObjCBool(false)
        let exists = FileManager.default.fileExists(atPath: fileName, isDirectory: &isDirectory)
        if !exists && isDirectory.boolValue {
            print("error: MeshStorageSettings, validateMeshFilePath, \(fileName), not exists")
            return false
        }
        
        return true
    }
}
