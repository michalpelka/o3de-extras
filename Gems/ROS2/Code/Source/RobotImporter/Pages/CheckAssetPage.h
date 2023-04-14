/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Math/Crc.h>
#include <AzCore/std/string/string.h>
#include <QLabel>
#include <QString>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QWizardPage>
#include <QVector>
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/std/containers/map.h>
#include <QTimer>
#endif

namespace ROS2
{
    class CheckAssetPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit CheckAssetPage(QWizard* parent);
        void ReportAsset(
            const AZ::Data::AssetId assetId,
            const QString& urdfPath,
            const QString& type,
            const QString& assetSourcePath,
            const AZ::Crc32& crc32,
            const QString& resolvedUrdfPath,
            const QString& productAsset);
        void ClearAssetsList();

        bool isComplete() const override;
    Q_SIGNALS:
        void UserRediscoverRequest();
    private:
        bool m_success;
        QTimer* refreshTimer;
        QTableWidget* m_table {};
        QTableWidgetItem* createCell(bool isOk, const QString& text);
        QPushButton* m_reload {};
        unsigned int m_missingCount;
        void SetTitle();
        AZStd::vector<AZ::Data::AssetId> assetsId;
        AZStd::unordered_set<AZ::Data::AssetId> processedAssets;
        QVector<QString> assetsPaths;
        void DoubleClickRow(int row, int col);
        void RefreshTimerElapsed();


    };
} // namespace ROS2
