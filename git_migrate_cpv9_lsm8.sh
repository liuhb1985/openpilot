#!/bin/bash

# ==========================================
# Git仓库迁移脚本
# Project: openpilot cpv9-lsm8
# Author: liuhb1985
# ==========================================

set -e

NEW_REPO="https://jihulab.com/liuhb1985/openpilot/cpv9-lsm8.git"
NEW_BRANCH="cpv9-lsm8"

echo "======================================"
echo " Git migrate to cpv9-lsm8"
echo "======================================"

# 检查是否Git工程
if [ ! -d ".git" ]; then
    echo "[INFO] 当前目录不是Git仓库"
    echo "[INFO] 初始化Git..."

    git init
fi


echo ""
echo "[1/6] 当前Git信息:"
git remote -v || true


echo ""
echo "[2/6] 删除旧remote..."

if git remote | grep -q origin; then
    git remote remove origin
    echo "旧origin已删除"
else
    echo "没有发现origin"
fi


echo ""
echo "[3/6] 创建新分支 ${NEW_BRANCH}"

# 如果已有分支则切换
if git branch | grep -q "${NEW_BRANCH}"; then
    git checkout ${NEW_BRANCH}
else
    git checkout -b ${NEW_BRANCH}
fi


echo ""
echo "[4/6] 添加新remote"

git remote add origin ${NEW_REPO}


echo ""
echo "[5/6] 提交代码"

git add .

# 判断是否有修改
if git diff --cached --quiet; then
    echo "没有新的修改"
else
    git commit -m "cpv9-lsm8 initial version"
fi


echo ""
echo "[6/6] 推送到Jihulab"

git push -u origin ${NEW_BRANCH}


echo ""
echo "======================================"
echo " 完成!"
echo " 新仓库:"
echo ${NEW_REPO}
echo " 分支:"
echo ${NEW_BRANCH}
echo "======================================"

git remote -v
git branch
