/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SourceAssetsStorage.h"
#include "RobotImporterUtils.h"
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/DataTypes/Groups/ISceneNodeGroup.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <SceneAPI/SceneData/Rules/UVsRule.h>
#include <SceneAPI/SceneData/Groups/MeshGroup.h>
#include <AzToolsFramework/Asset/AssetUtils.h>
#include <Source/Pipeline/MeshGroup.h> // PhysX/Code/Source/Pipeline/MeshGroup.h



namespace ROS2::Utils
{



    //! A helper class that do no extend PhysX::Pipeline::MeshGroup functionality, but gives neccessary setters.
    class UrdfPhysxMeshGroupHelper : public PhysX::Pipeline::MeshGroup
    {
    public:
        void SetIsDecomposeMeshes(bool decompose){
            m_decomposeMeshes = decompose;
        }
        void SetMeshExportMethod(PhysX::Pipeline::MeshExportMethod method){
            m_exportMethod = method;
        }
    };

    //! Returns supportered filenames by Asset Processor
    AZStd::vector<AZStd::string> GetSupportedExtensions()
    {
        struct Visitor
            : AZ::SettingsRegistryInterface::Visitor
        {
            void Visit(const AZ::SettingsRegistryInterface::VisitArgs&, AZStd::string_view value) override
            {
                m_supportedFileExtensions.emplace_back(value);
            }

            AZStd::vector<AZStd::string> m_supportedFileExtensions;
        };
        auto settingsRegistry = AZ::SettingsRegistry::Get();

        if (settingsRegistry == nullptr)
        {
            AZ_Warning("GetInterestingSourceAssetsCRC", false, "Global Settings Registry is not set.");
            return AZStd::vector<AZStd::string>();
        }

        using namespace AzToolsFramework::AssetUtils;
        Visitor assetImporterVisitor;
        settingsRegistry->Visit(assetImporterVisitor, AZ::SettingsRegistryInterface::FixedValueString("/O3DE/SceneAPI/AssetImporter/SupportedFileTypeExtensions"));
        return assetImporterVisitor.m_supportedFileExtensions;

    }

    /// Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZStd::string& filename)
    {
        auto fileSize = AZ::IO::SystemFile::Length(filename.c_str());
        fileSize = AZStd::min(fileSize, 1024ull); // limit crc computation to first kilobyte
        if (fileSize == 0)
        {
            return AZ::Crc32();
        }
        AZStd::vector<char> buffer;
        buffer.resize_no_construct(fileSize + 1);
        buffer[fileSize] = '\0';
        if (!AZ::IO::SystemFile::Read(filename.c_str(), buffer.data(), fileSize))
        {
            return AZ::Crc32();
        }
        AZ::Crc32 r;
        r.Add(buffer.data(), fileSize);
        return r;
    }

    AZStd::string GetProductAsset(const AZ::Data::AssetId& assetId , const AZ::TypeId typeId)
    {
        AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        bool ok{ false };
        AssetSysReqBus::BroadcastResult(
            ok, &AssetSysReqBus::Events::GetAssetsProducedBySourceUUID, assetId.m_guid, productsAssetInfo);
        if (ok)
        {
            for (auto& product : productsAssetInfo)
            {
                if (product.m_assetType == typeId)
                {
                    AZStd::string assetPath;
                    AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                        assetPath, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetPathById, product.m_assetId);
                    return assetPath;
                }
            }
        }
        return "";
    }


    AZStd::string GetModelProductAsset(const AZ::Data::AssetId& assetId)
    {
        return GetProductAsset(assetId, AZ::TypeId("{2C7477B6-69C5-45BE-8163-BCD6A275B6D8}")); //AZ::RPI::ModelAsset;
    }

    AZStd::string GetPhysXMeshProductAsset(const AZ::Data::AssetId& assetId)
    {
        return GetProductAsset(assetId, AZ::TypeId("{7A2871B9-5EAB-4DE0-A901-B0D2C6920DDB}")); //PhysX::Pipeline::MeshAsset
    }




    AZStd::unordered_map<AZ::Crc32, AvailableAsset> GetInterestingSourceAssetsCRC()
    {
        // connect to database API
        const AZStd::vector<AZStd::string> InterestingExtensions = GetSupportedExtensions();
        AZStd::unordered_map<AZ::Crc32, AvailableAsset> availableAssets;

        AzToolsFramework::AssetDatabase::AssetDatabaseConnection assetDatabaseConnection;
        if (!assetDatabaseConnection.OpenDatabase())
        {
            AZ_Warning("GetInterestingSourceAssetsCRC", false, "Cannot open database");
        }
        auto callback = [&availableAssets](AzToolsFramework::AssetDatabase::SourceDatabaseEntry& entry){
            using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
            AvailableAsset foundAsset;
            foundAsset.m_sourceID = entry.m_sourceID;
            foundAsset.m_sourceGuid = entry.m_sourceGuid;

            using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;

            // get source asset info
            bool sourceAssetFound{ false };
            AZ::Data::AssetInfo assetInfo;
            AZStd::string watchFolder;
            AssetSysReqBus::BroadcastResult(
                sourceAssetFound, &AssetSysReqBus::Events::GetSourceInfoBySourceUUID, entry.m_sourceGuid, assetInfo, watchFolder);
            if (!sourceAssetFound)
            {
                AZ_Warning("GetInterestingSourceAssetsCRC", false, "Cannot find source asset info for %s", entry.ToString().c_str());
                return true;
            }
            foundAsset.m_assetId = assetInfo.m_assetId;

            const auto fullSourcePath = AZ::IO::Path(watchFolder)/AZ::IO::Path(assetInfo.m_relativePath);

            foundAsset.m_sourceAssetRelativePath = assetInfo.m_relativePath;
            foundAsset.m_sourceAssetGlobalPath = fullSourcePath.String();

            AZ::Crc32 crc = Utils::GetFileCRC(foundAsset.m_sourceAssetGlobalPath);
            if (crc == AZ::Crc32(0))
            {
                AZ_Warning("GetInterestingSourceAssetsCRC", false, "Zero CRC for source asset %s", foundAsset.m_sourceAssetGlobalPath.c_str());
                return true;
            }
            // Todo Remove in review
            AZ_Printf("GetInterestingSourceAssetsCRC", "Found asset:");
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tm_sourceAssetRelativePath  : %s",foundAsset.m_sourceAssetRelativePath.c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tm_sourceAssetGlobalPath    : %s",foundAsset.m_sourceAssetGlobalPath.c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tm_productAssetRelativePath : %s",foundAsset.m_productAssetRelativePath.c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tm_sourceGuid               : %s",foundAsset.m_sourceGuid.ToString<AZStd::string>().c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tproductAsset (Visual)      : %s",GetModelProductAsset(foundAsset.m_assetId).c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tproductAsset (PhysX)       : %s",GetPhysXMeshProductAsset(foundAsset.m_assetId).c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tcrc                        : %d",crc);

            auto availableAssetIt = availableAssets.find(crc);
            if (availableAssetIt != availableAssets.end())
            {
                AZ_Warning("GetInterestingSourceAssetsCRC", false, "Asset already in database : %s ",  foundAsset.m_sourceAssetGlobalPath.c_str());
                AZ_Warning("GetInterestingSourceAssetsCRC", false, "Found asset : %s ", availableAssetIt->second.m_sourceAssetGlobalPath.c_str());
            }
            else
            {
                availableAssets.insert({crc, foundAsset});
            }
            return true;
        };

        for (auto &extension: InterestingExtensions){
            assetDatabaseConnection.QuerySourceLikeSourceName(extension.c_str(), AzToolsFramework::AssetDatabase::AssetDatabaseConnection::LikeType::EndsWith, callback);
        }
        return availableAssets;
    }


    UrdfAssetMap FindAssetsForUrdf(const AZStd::unordered_set<AZStd::string>& meshesFilenames, const AZStd::string& urdFilename)
    {
        auto enviromentalVariable = std::getenv("AMENT_PREFIX_PATH");
        AZ_Warning("UrdfAssetMap", enviromentalVariable, "AMENT_PREFIX_PATH is not found.");
        AZStd::string amentPrefixPath {enviromentalVariable};

        UrdfAssetMap urdfToAsset;
        for (const auto& t : meshesFilenames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = t;
            asset.m_resolvedUrdfPath = Utils::ResolveURDFPath(asset.m_urdfPath, urdFilename, amentPrefixPath);
            asset.m_urdfFileCRC = Utils::GetFileCRC(asset.m_resolvedUrdfPath);
            urdfToAsset.emplace(t, AZStd::move(asset));
        }

        const AZStd::unordered_map<AZ::Crc32, AvailableAsset> availableAssets = Utils::GetInterestingSourceAssetsCRC();

        // Search for suitable mappings by comparing checksum
        for (auto it = urdfToAsset.begin(); it != urdfToAsset.end(); it++)
        {
            Utils::UrdfAsset& asset = it->second;
            auto found_source_asset = availableAssets.find(asset.m_urdfFileCRC);
            if (found_source_asset != availableAssets.end())
            {
                asset.m_availableAssetInfo = found_source_asset->second;
            }
        }
        return urdfToAsset;
    }

    bool createSceneManifest(const AZStd::string sourceAssetPath, bool collider, bool visual)
    {
        const AZStd::string azMeshPath = sourceAssetPath;

        AZStd::shared_ptr<AZ::SceneAPI::Containers::Scene> scene;
        AZ::SceneAPI::Events::SceneSerializationBus::BroadcastResult(
            scene, &AZ::SceneAPI::Events::SceneSerialization::LoadScene, azMeshPath.c_str(), AZ::Uuid::CreateNull(), "");
        if (!scene)
        {
            AZ_Error("createSceneManifest", false, "Error loading collider. Invalid scene: %s", azMeshPath.c_str());
            return false;
        }

        AZ::SceneAPI::Containers::SceneManifest& manifest = scene->GetManifest();
        auto valueStorage = manifest.GetValueStorage();
        if (valueStorage.empty())
        {
            AZ_Error("createSceneManifest", false, "Error loading collider. Invalid value storage: %s", azMeshPath.c_str());
            return false;
        }

        // remove default configuration to avoid procedural prefab creation
        AZStd::vector<AZStd::shared_ptr<AZ::SceneAPI::DataTypes::IManifestObject>> toDelete;
        for (size_t i =0; i < manifest.GetEntryCount(); i++)
        {
            AZStd::shared_ptr<AZ::SceneAPI::DataTypes::IManifestObject> obj = manifest.GetValue(i);
            toDelete.push_back(obj);
        }

        for (auto obj : toDelete)
        {
            AZ_Printf("createSceneManifest", "Deleting %s", obj->RTTI_GetType().ToString<AZStd::string>().c_str());
            manifest.RemoveEntry(obj);
        }

        if (visual)
        {
            AZStd::shared_ptr<AZ::SceneAPI::SceneData::MeshGroup> sceneDataMeshGroup = AZStd::make_shared<AZ::SceneAPI::SceneData::MeshGroup>();

            // select all nodes to this mesh group
            AZ::SceneAPI::Utilities::SceneGraphSelector::SelectAll(scene->GetGraph(), sceneDataMeshGroup->GetSceneNodeSelectionList());

            // enable auto-generation of UVs
            sceneDataMeshGroup->GetRuleContainer().AddRule(AZStd::make_shared<AZ::SceneAPI::SceneData::UVsRule>());

            manifest.AddEntry(sceneDataMeshGroup);

        }

        if (collider)
        {
            AZStd::shared_ptr<UrdfPhysxMeshGroupHelper> physxDataMeshGroup = AZStd::make_shared<UrdfPhysxMeshGroupHelper>();
            physxDataMeshGroup->SetIsDecomposeMeshes(true);
            physxDataMeshGroup->SetMeshExportMethod(PhysX::Pipeline::MeshExportMethod::Convex);

            // select all nodes to this mesh group
            AZ::SceneAPI::Utilities::SceneGraphSelector::SelectAll(scene->GetGraph(), physxDataMeshGroup->GetSceneNodeSelectionList());

            manifest.AddEntry(physxDataMeshGroup);
        }

        // Update assetinfo
        AZ::SceneAPI::Events::ProcessingResultCombiner result;
        AZ::SceneAPI::Events::AssetImportRequestBus::BroadcastResult(
            result,
            &AZ::SceneAPI::Events::AssetImportRequest::UpdateManifest,
            *scene,
            AZ::SceneAPI::Events::AssetImportRequest::ManifestAction::Update,
            AZ::SceneAPI::Events::AssetImportRequest::RequestingApplication::Editor);

        if (result.GetResult() != AZ::SceneAPI::Events::ProcessingResult::Success)
        {
            AZ_TracePrintf("createSceneManifest", "Scene updated\n");
            return false;
        }
        auto assetInfoFilePath = AZ::IO::Path{ azMeshPath };
        assetInfoFilePath.Native() += ".assetinfo";
        scene->GetManifest().SaveToFile(assetInfoFilePath.c_str());

        AZ_Printf("createSceneManifest", "Saving scene manifest to %s\n", assetInfoFilePath.c_str());

        return true;
    }
} // namespace ROS2::Utils
