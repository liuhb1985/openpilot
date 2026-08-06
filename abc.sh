#!/bin/bash

REMOTE="https://jihulab.com/liuhb1985/openpilot.git"
BRANCH="cpv9-wip"

echo "Remove old git..."
rm -rf .git

echo "Init git..."
git init

echo "Init LFS..."
git lfs install

echo "Add files..."
git add .

echo "Commit..."
git commit -m "cpv9-lsm8 initial"

echo "Add remote..."
git remote add origin $REMOTE

echo "Create branch..."
git branch -M $BRANCH

echo "Push..."
git push -u origin $BRANCH

echo "DONE"
