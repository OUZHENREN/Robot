#!/bin/bash

SRC="/home/yff/environment_point_cloud_fusion"
DST="/mnt/hgfs/Model/environment_point_cloud_fusion"
LOG_FILE="/home/yff/elite_ros_ws/sync_environment_pointcloud.log"

mkdir -p "$DST"

echo "[$(date '+%F %T')] Sync service started" | tee -a "$LOG_FILE"
echo "[$(date '+%F %T')] Source: $SRC" | tee -a "$LOG_FILE"
echo "[$(date '+%F %T')] Dest:   $DST" | tee -a "$LOG_FILE"

# 启动时先同步一次
rsync -av --delete "$SRC"/ "$DST"/ >> "$LOG_FILE" 2>&1
echo "[$(date '+%F %T')] Initial sync completed" | tee -a "$LOG_FILE"

# 持续监听目录变化
inotifywait -m -r -e create -e modify -e delete -e move "$SRC" | while read -r path action file; do
    echo "[$(date '+%F %T')] Detected change: path=$path action=$action file=$file" | tee -a "$LOG_FILE"

    rsync -av --delete "$SRC"/ "$DST"/ >> "$LOG_FILE" 2>&1

    if [ $? -eq 0 ]; then
        echo "[$(date '+%F %T')] Sync completed successfully" | tee -a "$LOG_FILE"
    else
        echo "[$(date '+%F %T')] Sync failed" | tee -a "$LOG_FILE"
    fi
done
