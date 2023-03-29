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

namespace ROS2::Utils
{
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


////        if (!assetDatabaseConnection.OpenDatabase())
////        {
////            AZ_Warning("GetInterestingSourceAssetsCRC", false, "Cannot open database");
////        }
////        AZStd::string productAssetPath;
////        auto callback = [&](AzToolsFramework::AssetDatabase::ProductDatabaseEntry& entry){
////            if( entry.m_assetType == AZ::TypeId("{2C7477B6-69C5-45BE-8163-BCD6A275B6D8}")) // AZ::RPI::ModelAsset
////            {
////                AZ_Printf("getProductAsset", "%s", entry.ToString().c_str());
////                productAssetPath = entry.m_productName;
////                return false;
////            }
////            return true;
////        };
////        assetDatabaseConnection.QueryProductBySourceID( asset.m_sourceID,callback);
////        return productAssetPath;
//    }
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

        const AZStd::vector<AZStd::string> kInterestingExtensions{ ".dae", ".stl", ".obj", ".fbx" };
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

        for (auto &extension: kInterestingExtensions){
            assetDatabaseConnection.QuerySourceLikeSourceName(extension.c_str(), AzToolsFramework::AssetDatabase::AssetDatabaseConnection::LikeType::EndsWith, callback);
        }
        return availableAssets;
    }


    UrdfAssetMap FindAssetsForUrdf(const AZStd::unordered_set<AZStd::string>& meshesFilenames, const AZStd::string& urdFilename)
    {
        UrdfAssetMap urdfToAsset;
        for (const auto& t : meshesFilenames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = t;
            asset.m_resolvedUrdfPath = Utils::ResolveURDFPath(asset.m_urdfPath, urdFilename);
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

        //disable procedural prefab generation
        AZStd::vector<AZStd::shared_ptr<AZ::SceneAPI::DataTypes::IManifestObject>> toDelete;
        for (size_t i =0; i < manifest.GetEntryCount(); i++)
        {
            AZStd::shared_ptr<AZ::SceneAPI::DataTypes::IManifestObject> obj = manifest.GetValue(i);
            if (obj->RTTI_IsTypeOf(AZ::TypeId("99FE3C6F-5B55-4D8B-8013-2708010EC715"))) // PrefabBuilder/PrefabGroup/PrefabGroup
            {
                toDelete.push_back(obj);
            }
            if (!visual && obj->RTTI_IsTypeOf(AZ::TypeId("07B356B7-3635-40B5-878A-FAC4EFD5AD86"))) // SceneAPI/SceneData/Groups/MeshGroup
            {
                toDelete.push_back(obj);
            }
            if (!collider&&obj->RTTI_IsTypeOf(AZ::TypeId("5B03C8E6-8CEE-4DA0-A7FA-CD88689DD45B"))) // PhysX/Code/Source/Pipeline/MeshGroup
            {
                toDelete.push_back(obj);
            }

        }
        for (auto obj : toDelete)
        {
            AZ_Printf("createSceneManifest", "Deleting %s", obj->RTTI_GetType().ToString<AZStd::string>().c_str());
            manifest.RemoveEntry(obj);
        }

        auto view = AZ::SceneAPI::Containers::MakeDerivedFilterView<AZ::SceneAPI::DataTypes::ISceneNodeGroup>(valueStorage);
        if (view.empty())
        {
            AZ_Error("createSceneManifest", false, "Error loading collider. Invalid node views: %s", azMeshPath.c_str());
            return false;
        }

        for (AZ::SceneAPI::DataTypes::ISceneNodeGroup& mg : view)
        {
            AZ_Printf("createSceneManifest", "aaa : %s %s" ,mg.GetName().c_str(), mg.RTTI_GetType().ToString<AZStd::string>().c_str());
            AZ::SceneAPI::Utilities::SceneGraphSelector::SelectAll(scene->GetGraph(), mg.GetSceneNodeSelectionList());
        }


        // Update scene with all nodes selected
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
        if (collider)
        {
            // Set export method to convex mesh
            auto readOutcome = AZ::JsonSerializationUtils::ReadJsonFile(assetInfoFilePath.c_str());
            if (!readOutcome.IsSuccess())
            {
                AZ_Error(
                    "createSceneManifest",
                    false,
                    "Could not read %s with %s",
                    assetInfoFilePath.c_str(),
                    readOutcome.GetError().c_str());
                return false;
            }
            rapidjson::Document assetInfoJson = readOutcome.TakeValue();
            auto manifestObject = assetInfoJson.GetObject();
            auto valuesIterator = manifestObject.FindMember("values");
            if (valuesIterator == manifestObject.MemberEnd())
            {
                AZ_Error(
                    "createSceneManifest", false, "Invalid json file: %s (Missing 'values' node)", assetInfoFilePath.c_str());
                return false;
            }

            constexpr AZStd::string_view physXMeshGroupType = "{5B03C8E6-8CEE-4DA0-A7FA-CD88689DD45B} MeshGroup";
            auto valuesArray = valuesIterator->value.GetArray();
            for (auto& value : valuesArray)
            {
                auto object = value.GetObject();

                auto physXMeshGroupIterator = object.FindMember("$type");
                if (AZ::StringFunc::Equal(physXMeshGroupIterator->value.GetString(), physXMeshGroupType))
                {
                    value.AddMember(rapidjson::StringRef("export method"), rapidjson::StringRef("1"), assetInfoJson.GetAllocator());
                }
            }

            auto saveOutcome = AZ::JsonSerializationUtils::WriteJsonFile(assetInfoJson, assetInfoFilePath.c_str());
            if (!saveOutcome.IsSuccess())
            {
                AZ_Error(
                    "createSceneManifest",
                    false,
                    "Could not save %s with %s",
                    assetInfoFilePath.c_str(),
                    saveOutcome.GetError().c_str());
                return false;
            }
        }
        return true;
    }
} // namespace ROS2::Utils
