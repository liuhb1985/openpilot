#!/usr/bin/bash
# NOTE: -e in the shebang is NOT applied when the script is invoked as
# `bash build_nobigmodel.sh`, so set it explicitly here. Without it a failed
# `git push` would fall through and the script would report "Published" even
# though nothing was pushed.
set -e

BUILD_DIR=/data/openpilot
cd $BUILD_DIR

# ---------------------------------------------------------------------------
# Prebuilt release WITHOUT the eGPU "big model" artifacts.
#
# Same as build_release.sh (rewrites git history with rm -rf .git, force-pushes
# the egpucp branch and deletes release/), plus it keeps two things out of the
# published branch:
#
#   1. eGPU big-model artifacts
#      openpilot/selfdrive/modeld/models/big_driving_*      (~750MB)
#      openpilot/selfdrive/modeld/models/.big_model_build_stamp
#      They are archived to /data/bigmodel_backup_<date>.tar.gz first, then
#      moved out of the tree, then restored locally after the push.
#
#   2. local backup dirs that would otherwise be swept in by `git add -f .`
#      (see LOCAL_EXCLUDE_PATHS below, e.g. amapnavi_bak/)
#      Also moved out and restored afterwards.
#
# Why a release without the big model still runs:
#   launch_chffrplus.sh -> prepare_big_model_if_needed() calls
#   helpers.usbgpu_present(), which looks for the USB eGPU bridge
#   (VID:PID ADD1:0001 / 3801:0001 at >=5Gbps). With no eGPU attached the whole
#   path returns early, and invalidate_modeld_build_if_needed() skips the
#   big-model stamp check because BIG_MODEL_SHA stays empty. The small models
#   (driving_tinygrad.pkl.chunk*, dmonitoring_model_*, dm_warp_*) and
#   tg_input_devices.json are kept, which is what that check actually requires.
#
# Restoring the big model on another device (needs the tar.gz, e.g. over scp):
#   scp /data/bigmodel_backup_XXXX.tar.gz comma@<device>:/data/
#   ssh comma@<device> 'tar xzf /data/bigmodel_backup_XXXX.tar.gz -C /data/openpilot'
#   then reboot.  (Paths inside the archive are relative to /data/openpilot.)
#
# Set SKIP_CONFIRM=1 to run unattended, TOKEN=xxx to avoid the
# interactive credential prompt on push.
# ---------------------------------------------------------------------------

# Local paths (relative to $BUILD_DIR) that must NOT be published. Space
# separated. Everything listed here is moved aside and restored afterwards.
LOCAL_EXCLUDE_PATHS="amapnavi_bak"

if [ -z "$SKIP_CONFIRM" ]; then
  echo "This will: back up + exclude eGPU big-model artifacts and local backup"
  echo "            dirs, rm -rf .git, force-push branch egpucp to jihulab,"
  echo "            delete release/."
  read -r -p "Type yes to continue: " CONFIRM
  if [ "$CONFIRM" != "yes" ]; then
    echo "Aborted."
    exit 1
  fi
fi

echo "==> Stopping openpilot"
pkill -f "[c]omma.sh" 2>/dev/null || true
pkill -f "[l]aunch_chffrplus.sh" 2>/dev/null || true
pkill -f "[m]anager.py" 2>/dev/null || true
tmux kill-server 2>/dev/null || true
sleep 2
echo "    openpilot stopped"

echo "==> Backing up .git and release/"
BK="/data/pre_release_backup_$(date +%m%d_%H%M)"
mkdir -p "$BK"
cp -a "$BUILD_DIR/.git" "$BK/git"
cp -a "$BUILD_DIR/release" "$BK/release"
echo "    saved to $BK"

# ---------------------------------------------------------------------------
# eGPU big-model artifacts: archive, then move out of the tree.
# ---------------------------------------------------------------------------
BIG_MODEL_DIR="$BUILD_DIR/openpilot/selfdrive/modeld/models"
BIG_MODEL_STAMP="$BIG_MODEL_DIR/.big_model_build_stamp"
BIG_KEEP="/data/bigmodel_pub_keep"
BIG_TAR="/data/bigmodel_backup_$(date +%m%d_%H%M).tar.gz"

echo "==> Excluding eGPU big-model artifacts"
rm -rf "$BIG_KEEP"
mkdir -p "$BIG_KEEP"
if compgen -G "$BIG_MODEL_DIR/big_driving_*" >/dev/null 2>&1 || [ -f "$BIG_MODEL_STAMP" ]; then
  # Relative paths: extracting with -C /data/openpilot restores every file
  # exactly where it was. Build the member list defensively so a missing piece
  # (e.g. no stamp) cannot make tar fail under `set -e`.
  BIG_TAR_LIST=""
  compgen -G "$BIG_MODEL_DIR/big_driving_*" >/dev/null 2>&1 && \
    BIG_TAR_LIST="openpilot/selfdrive/modeld/models/big_driving_*"
  [ -f "$BIG_MODEL_STAMP" ] && \
    BIG_TAR_LIST="$BIG_TAR_LIST openpilot/selfdrive/modeld/models/.big_model_build_stamp"
  # Space pre-check: the archive is written next to the tree, so make sure
  # /data has room (gzip lands around half the raw size) before we start.
  BIG_RAW_MB=$(du -sm "$BIG_MODEL_DIR"/big_driving_* 2>/dev/null | awk '{s+=$1} END {print s+0}')
  BIG_FREE_MB=$(df -Pm /data | awk 'NR==2 {print $4}')
  BIG_NEED_MB=$((BIG_RAW_MB / 2 + 200))
  echo "    source ${BIG_RAW_MB}MB, /data free ${BIG_FREE_MB}MB, need ~${BIG_NEED_MB}MB"
  if [ "$BIG_FREE_MB" -lt "$BIG_NEED_MB" ]; then
    echo "    ERROR: not enough space under /data to archive the big model."
    echo "           free up space or set BIG_MODEL_TAR_DIR to another mount."
    exit 1
  fi
  tar czf "$BIG_TAR" -C "$BUILD_DIR" --wildcards $BIG_TAR_LIST
  echo "    archive: $BIG_TAR ($(du -h "$BIG_TAR" | cut -f1))"

  mv "$BIG_MODEL_DIR"/big_driving_* "$BIG_KEEP"/ 2>/dev/null || true
  mv "$BIG_MODEL_STAMP" "$BIG_KEEP"/ 2>/dev/null || true
  echo "    moved out of tree -> $BIG_KEEP (restored after publish)"
else
  echo "    none found, skipping"
fi

# ---------------------------------------------------------------------------
# Local backup dirs/files that must not be published either.
# `git add -f .` ignores .gitignore, so they have to leave the tree.
# ---------------------------------------------------------------------------
LOCAL_KEEP="/data/release_local_keep"
echo "==> Excluding local backup paths: $LOCAL_EXCLUDE_PATHS"
rm -rf "$LOCAL_KEEP"
mkdir -p "$LOCAL_KEEP"
MOVED_LOCAL=""
for p in $LOCAL_EXCLUDE_PATHS; do
  if [ -e "$BUILD_DIR/$p" ]; then
    mkdir -p "$LOCAL_KEEP/$(dirname "$p")"
    mv "$BUILD_DIR/$p" "$LOCAL_KEEP/$p"
    MOVED_LOCAL="$MOVED_LOCAL $p"
    echo "    $p -> $LOCAL_KEEP/$p"
  fi
done
[ -n "$MOVED_LOCAL" ] || echo "    none present"

echo "==> Moving pydeps out of the tree"
if [ -d "$BUILD_DIR/pydeps" ]; then
  rm -rf /data/pydeps_pub_keep
  mv "$BUILD_DIR/pydeps" /data/pydeps_pub_keep
  echo "    pydeps -> /data/pydeps_pub_keep (restored after publish)"
fi

rm -rf .git
git init
git remote add origin https://jihulab.com/fishop/openpilot.git

# Optional token so the push does not prompt for credentials. It is stripped
# from the remote again right after the push.
if [ -n "$TOKEN" ]; then
  git remote set-url origin "https://fishop:${TOKEN}@jihulab.com/fishop/openpilot.git"
fi

# in the directory
cd $BUILD_DIR

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name 'moc_*' -delete
find . -name '__pycache__' -delete
rm -rf .sconsign.dblite Jenkinsfile release/
#rm -f openpilot/selfdrive/modeld/models/*.onnx
# drop the legacy stamp inside modeld/; it is regenerated at repo root below
rm -f openpilot/selfdrive/modeld/models/.build_stamp

# ship the prebuilt release WITHOUT the .onnx model inputs (the tinygrad .pkl
# artifacts + prebuilt marker are enough). Keep the files on disk so the device
# can still rebuild if ever needed; just don't commit them.
# NOTA BENE: `git add -f .` below overrides .gitignore, so these entries are
# only a safety net; the real exclusion happens by moving files out above.
{
  echo 'openpilot/selfdrive/modeld/models/*.onnx'
  echo 'openpilot/selfdrive/modeld/models/big_driving_*'
  echo 'openpilot/selfdrive/modeld/models/.big_model_build_stamp'
  for p in $LOCAL_EXCLUDE_PATHS; do
    echo "$p"
    echo "$p/"
  done
} > .gitignore

find third_party/ -name '*x86*' -exec rm -r {} +
find third_party/ -name '*Darwin*' -exec rm -r {} +

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .

VERSION="carrot_v$(date +%y%m%d)"
git commit -m $VERSION
git branch -m "egpucp"

# Recompute .build_stamp against the exact HEAD that will be pushed.
# launch_chffrplus.sh compares this stamp on every boot; if it doesn't match
# the pushed HEAD, FORCE_REBUILD=1 and the device tries to recompile the
# models. But the .onnx inputs are not shipped in this prebuilt release, so the
# build fails and the device hangs. Regenerate the stamp here so the release
# always matches its own HEAD.
# NOTE: the stamp value is the git tree hash of openpilot/selfdrive/modeld, so
# the stamp file itself MUST live outside that tree (repo root). If it lived
# inside modeld/, changing it would change the very hash it records, and the
# stamp could never match after commit.
STAMP="$(git rev-parse HEAD:openpilot/selfdrive/modeld HEAD:tinygrad_repo HEAD:openpilot/common/file_chunker.py | tr '\n' ':')"
echo -n "$STAMP" > .build_stamp
git add -f .build_stamp
git commit -m "${VERSION}-stamp"

# ---------------------------------------------------------------------------
# Safety check: the published branch must not carry anything we excluded.
# ---------------------------------------------------------------------------
echo "==> Verifying the release carries no excluded files"
FAILED=0
if git ls-files | grep -q 'modeld/models/big_driving_'; then
  echo "    ERROR: big-model chunks are still tracked:"
  git ls-files | grep 'modeld/models/big_driving_' | head -5
  FAILED=1
fi
if git ls-files | grep -q 'modeld/models/\.big_model_build_stamp'; then
  echo "    ERROR: .big_model_build_stamp is still tracked"
  FAILED=1
fi
for p in $LOCAL_EXCLUDE_PATHS; do
  if git ls-files | grep -q "^$p/"; then
    echo "    ERROR: excluded local path is still tracked: $p"
    git ls-files | grep "^$p/" | head -5
    FAILED=1
  fi
done
if [ "$FAILED" = "1" ]; then
  echo "    aborting push; nothing restored yet. Local keep dirs:"
  echo "      $BIG_KEEP"
  echo "      $LOCAL_KEEP"
  exit 1
fi
echo "    ok: none tracked"

# Capture the result instead of letting `set -e` abort here: the restore steps
# below must always run, otherwise the big model / pydeps / backup dirs would be
# left in /data/*_keep and the device would be broken.
PUSH_OK=1
git push -f origin "egpucp" || PUSH_OK=0

# ---------------------------------------------------------------------------
# Post-publish: strip the token from the remote and restore local runtime deps.
# ---------------------------------------------------------------------------
if [ -n "$TOKEN" ]; then
  git remote set-url origin https://jihulab.com/fishop/openpilot.git
fi

echo "==> Restoring eGPU big-model artifacts (this device only)"
if [ -n "$(ls -A "$BIG_KEEP" 2>/dev/null)" ]; then
  mv "$BIG_KEEP"/big_driving_* "$BIG_MODEL_DIR"/ 2>/dev/null || true
  mv "$BIG_KEEP"/.big_model_build_stamp "$BIG_MODEL_DIR"/ 2>/dev/null || true
  rmdir "$BIG_KEEP" 2>/dev/null || true
  echo "    restored from $BIG_KEEP"
fi

echo "==> Restoring local backup paths"
for p in $MOVED_LOCAL; do
  if [ -e "$LOCAL_KEEP/$p" ]; then
    mkdir -p "$(dirname "$BUILD_DIR/$p")"
    mv "$LOCAL_KEEP/$p" "$BUILD_DIR/$p"
    echo "    $p restored"
  fi
done
rm -rf "$LOCAL_KEEP" 2>/dev/null || true

echo "==> Restoring pydeps"
if [ -d /data/pydeps_pub_keep ]; then
  mv /data/pydeps_pub_keep "$BUILD_DIR/pydeps"
  echo "    pydeps restored"
fi

if [ "$PUSH_OK" != "1" ]; then
  echo ""
  echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  echo "!! PUSH FAILED - nothing was published to jihulab."
  echo "!!   本地文件（大模型 / pydeps / amapnavi_bak）已恢复，可正常使用。"
  echo "!!   常见原因：jihulab 未配置凭据。可先 git config credential.helper store"
  echo "!!   或设置 TOKEN=<token> 后重跑本脚本。"
  echo "!!   注意：本次已 rm -rf .git 并重建为 egpucp 分支，"
  echo "!!         如需回到原来的分支，可从 $BK 恢复 .git。"
  echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  exit 1
fi

echo ""
echo "Published branch egpucp to jihulab as $VERSION (without eGPU big model)."
echo "Pre-publish backup: $BK"
echo "Big-model archive:  $BIG_TAR"
echo "  -> restore on this device: already restored"
echo "  -> restore elsewhere:      scp it to /data/ and run"
echo "                             tar xzf $(basename "$BIG_TAR") -C /data/openpilot"
echo "Now reboot to restart openpilot:  sudo reboot"
