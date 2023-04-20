/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CheckAssetPage.h"
#include <AzCore/Math/MathStringConversions.h>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QPushButton>
#include "AzFramework/Asset/AssetSystemBus.h"
#include "../Utils/SourceAssetsStorage.h"

namespace ROS2
{

    CheckAssetPage::CheckAssetPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(true)
        , m_missingCount(0)
    {
        m_table = new QTableWidget(parent);
        m_reload = new QPushButton(tr("Rediscover meshes"),parent);
        SetTitle();
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_reload);
        this->connect(m_reload, &QPushButton::pressed, this, &CheckAssetPage::UserRediscoverRequest);
        layout->addWidget(m_table);
        m_table->setEnabled(true);
        m_table->setAlternatingRowColors(true);
        m_table->setMinimumHeight(500);
        m_table->setMinimumWidth(1000);
        m_table->horizontalHeader()->setStretchLastSection(true);
        m_table->setCornerButtonEnabled(false);
        m_table->setSortingEnabled(false);
        m_table->setColumnCount(5);
        m_table->setShowGrid(true);
        m_table->setMouseTracking(true);
        m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_table->setSelectionMode(QAbstractItemView::SingleSelection);
        // Set the header items.
        QTableWidgetItem* headerItem = new QTableWidgetItem(tr("URDF mesh path"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(0, headerItem);
        headerItem = new QTableWidgetItem(tr("Resolved mesh from URDF"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(1, headerItem);
        headerItem = new QTableWidgetItem(tr("Type"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(2, headerItem);
        headerItem = new QTableWidgetItem(tr("Source asset"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(3, headerItem);
        headerItem = new QTableWidgetItem(tr("Product asset"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(4, headerItem);
        m_table->horizontalHeader()->resizeSection(0, 200);
        m_table->horizontalHeader()->resizeSection(1, 350);
        m_table->horizontalHeader()->resizeSection(2, 50);
        m_table->horizontalHeader()->resizeSection(3, 400);
        m_table->horizontalHeader()->resizeSection(4, 400);
        m_table->verticalHeader()->hide();
        connect(m_table, &QTableWidget::cellDoubleClicked, this, &CheckAssetPage::DoubleClickRow);
        this->setLayout(layout);
        refreshTimer = new QTimer(this);
        refreshTimer->setInterval(250);
        refreshTimer->setSingleShot(false);
        connect(refreshTimer, &QTimer::timeout, this, &CheckAssetPage::RefreshTimerElapsed);
    }

    void CheckAssetPage::SetTitle()
    {
        if (m_missingCount == 0)
        {
            setTitle(tr("Resolved meshes"));
        }
        else
        {
            setTitle(tr("There are ") + QString::number(m_missingCount) + tr(" unresolved meshes"));
        }
    }

    bool CheckAssetPage::isComplete() const
    {
        return m_success;
    };

    void CheckAssetPage::ReportAsset(
        const AZ::Uuid assetUuid,
        const QString& urdfPath,
        const QString& type,
        const QString& assetSourcePath,
        const AZ::Crc32& crc32,
        const QString& resolvedUrdfPath,
        const QString& productAsset)
    {
        int i = m_table->rowCount();
        m_table->setRowCount(i + 1);

        bool isOk = !assetSourcePath.isEmpty();
        if (!isOk)
        {
            m_missingCount++;
        }
        SetTitle();
        AZStd::string crcStr = AZStd::to_string(crc32);
        QTableWidgetItem* p = createCell(isOk, urdfPath);
        p->setToolTip(tr("CRC for file : ") + QString::fromUtf8(crcStr.data(), crcStr.size()));
        m_table->setItem(i, 0, p);
        m_table->setItem(i, 1, createCell(isOk, resolvedUrdfPath));
        m_table->setItem(i, 2, createCell(isOk, type));
        m_table->setItem(i, 3, createCell(isOk, assetSourcePath));
        m_table->setItem(i, 4, createCell(false, productAsset));
        m_assetsPaths.push_back(assetSourcePath);
        m_assetsUuids.push_back(assetUuid);
    }

    QTableWidgetItem* CheckAssetPage::createCell(bool isOk, const QString& text)
    {
        QTableWidgetItem* p = new QTableWidgetItem(text);
        if (!isOk)
        {
            p->setBackground(Qt::darkRed);
        }
        p->setToolTip(text);
        p->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        return p;
    }

    void CheckAssetPage::ClearAssetsList()
    {
        m_assetsUuids.clear();
        m_processedAssets.clear();
        m_assetsPaths.clear();
        m_table->setRowCount(0);
        m_missingCount = 0;
        refreshTimer->start();
    }

    void CheckAssetPage::DoubleClickRow(int row, int col){
        AZ_Printf("CheckAssetPage", "Clicked on row", row);
        if (row < m_assetsPaths.size())
        {
            AZStd::string path (m_assetsPaths[row].toUtf8().data());
            AzFramework::AssetSystemRequestBus::Broadcast(&AzFramework::AssetSystem::AssetSystemRequests::ShowInAssetProcessor, path);
        }
    }

    void CheckAssetPage::RefreshTimerElapsed(){
        for (int i =0; i < m_assetsUuids.size(); i++)
        {
            const AZ::Uuid &assetUuid  = m_assetsUuids[i];
            if (m_processedAssets.contains(assetUuid))
            {
                continue;
            }
            const AZStd::string productRelPathVisual = Utils::GetModelProductAsset(assetUuid);
            const AZStd::string productRelPathCollider = Utils::GetPhysXMeshProductAsset(assetUuid);
            QString text = QString::fromUtf8(productRelPathVisual.data(), productRelPathVisual.size())+ " " +
                QString::fromUtf8(productRelPathCollider.data(), productRelPathCollider.size());
            bool isOk = !productRelPathVisual.empty() || !productRelPathCollider.empty();
            if (isOk){
                m_processedAssets.insert(assetUuid);
                m_table->setItem(i, 4, createCell(true, text));
                m_processedAssets.insert(assetUuid);
            }
        }
    }
} // namespace ROS2
