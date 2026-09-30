#!/bin/bash

# ==========================================
# Git仓库迁移脚本
# Project: openpilot cpv9-lsm8
# Author: liuhb1985
# ==========================================

set -e

# ===============================
# 配置区域
# ===============================

NEW_REPO="https://jihulab.com/liuhb1985/openpilot.git"
NEW_BRANCH="cpv9-ch347"

GIT_USER="liuhb1985"
GIT_EMAIL="liuhb1985@gmail.com"


echo "======================================"
echo " Git migrate to ${NEW_BRANCH}"
echo "======================================"


# ===============================
# 检查Git仓库
# ===============================

if [ ! -d ".git" ]; then
    echo "[INFO] 当前目录不是Git仓库"
    echo "[INFO] 初始化Git..."

    git init
fi


# ===============================
# Git身份配置
# ===============================

echo ""
echo "[0/6] 检查Git用户信息"

git config user.name >/dev/null 2>&1 || \
git config user.name ${GIT_USER}

git config user.email >/dev/null 2>&1 || \
git config user.email ${GIT_EMAIL}


echo "Git user:"
git config user.name
git config user.email



# ===============================
# 查看旧remote
# ===============================

echo ""
echo "[1/6] 当前Git信息:"

git remote -v || true



# ===============================
# 删除旧remote
# ===============================

echo ""
echo "[2/6] 删除旧remote..."

if git remote | grep -q origin; then

    git remote remove origin

    echo "旧origin已删除"

else

    echo "没有发现origin"

fi



# ===============================
# 创建/切换分支
# ===============================

echo ""
echo "[3/6] 创建新分支 ${NEW_BRANCH}"


if git show-ref --verify --quiet refs/heads/${NEW_BRANCH}
then

    echo "分支已存在，切换到 ${NEW_BRANCH}"

    git checkout ${NEW_BRANCH}

else

    echo "创建分支 ${NEW_BRANCH}"

    git checkout -b ${NEW_BRANCH}

fi



# ===============================
# 添加新remote
# ===============================

echo ""
echo "[4/6] 添加新remote"


git remote add origin ${NEW_REPO}


echo "Remote:"
git remote -v



# ===============================
# 提交代码
# ===============================

echo ""
echo "[5/6] 提交代码"


git add .


if git diff --cached --quiet
then

    echo "没有新的修改"

else

    git commit -m "${NEW_BRANCH} initial version"

fi



# ===============================
# 推送
# ===============================

echo ""
echo "[6/6] 推送到Jihulab"


git push -u origin ${NEW_BRANCH}



echo ""
echo "======================================"
echo " 完成!"
echo "======================================"

echo "新仓库:"
echo ${NEW_REPO}

echo "分支:"
echo ${NEW_BRANCH}


echo ""
echo "当前remote:"
git remote -v


echo ""
echo "当前branch:"
git branch
