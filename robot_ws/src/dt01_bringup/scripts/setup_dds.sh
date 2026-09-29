#!/usr/bin/env bash
# ============================================================
# DT-01 多机通信配置（车 <-> 上位机）
#
# 为什么不能只 export ROS_DOMAIN_ID：
#   DDS 靠多播发现对端。家用路由器/4G 路由器普遍对多播限速甚至直接丢弃，
#   结果是"两边 ROS_DOMAIN_ID 一样，还是互相看不见话题"。
#   必须给 CycloneDDS 一份配置文件：指定网卡 + 显式 Peer 单播 + 数据走单播。
#
# 用法（两台机器都要各跑一次）：
#   交互式：bash setup_dds.sh
#   非交互：bash setup_dds.sh --role robot --iface wlan0 --peer 192.168.31.100
#           bash setup_dds.sh --role host  --iface wlp3s0 --peer 192.168.31.11
#   体检：  bash setup_dds.sh --check
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/../config/cyclonedds.xml.template"
TARGET="${HOME}/cyclonedds.xml"
BASHRC="${HOME}/.bashrc"
DOMAIN_ID=42

INFO(){ echo -e "\033[32m[INFO]\033[0m $*"; }
WARN(){ echo -e "\033[33m[WARN]\033[0m $*"; }
ERR(){  echo -e "\033[31m[ERR]\033[0m $*"; }

usage() {
    cat <<EOF
用法：
  bash setup_dds.sh                              # 交互式
  bash setup_dds.sh --role robot --iface wlan0 --peer 192.168.31.100
  bash setup_dds.sh --role host  --iface wlp3s0 --peer 192.168.31.11
  bash setup_dds.sh --check                      # 体检当前配置
  bash setup_dds.sh --domain 42 ...              # 自定义 ROS_DOMAIN_ID（默认 42）

参数：
  --role   robot|host   本机角色（robot=工控机，host=你的电脑）
  --iface  <网卡名>     用于 ROS 通信的网卡，如 wlan0 / wlp3s0 / eth0
  --peer   <IP>         对端 IP
  --domain <0-232>      ROS_DOMAIN_ID，两台机器必须一致
EOF
}

# ---------------- 参数解析 ----------------
ROLE=""; IFACE=""; PEER=""
while [ $# -gt 0 ]; do
    case "$1" in
        --role)   ROLE="$2"; shift 2;;
        --iface)  IFACE="$2"; shift 2;;
        --peer)   PEER="$2"; shift 2;;
        --domain) DOMAIN_ID="$2"; shift 2;;
        --check)  ROLE="__check__"; shift;;
        -h|--help) usage; exit 0;;
        *) ERR "未知参数: $1"; usage; exit 1;;
    esac
done

# ---------------- 体检模式 ----------------
if [ "$ROLE" = "__check__" ]; then
    echo "=============== DDS 配置体检 ==============="
    echo "--- 网卡与 IP ---"
    ip -o -4 addr show 2>/dev/null | awk '{printf "  %-12s %s\n", $2, $4}'
    echo "--- 环境变量 ---"
    for v in ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI; do
        echo "  ${v} = ${!v:-<未设置>}"
    done
    echo "--- RMW 实现 ---"
    if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
        echo "  ✓ rmw_cyclonedds_cpp 已安装"
    else
        echo "  ✗ rmw_cyclonedds_cpp 未安装！执行："
        echo "      sudo apt install -y ros-humble-rmw-cyclonedds-cpp"
    fi
    echo "  ROS_DISTRO = ${ROS_DISTRO:-<未 source ROS>}"
    echo "--- 配置文件 ---"
    if [ -f "${TARGET}" ]; then
        echo "  ✓ ${TARGET} 存在"
        grep -E "NetworkInterface|Peer address|AllowMulticast" "${TARGET}" \
            | sed 's/^/    /'
    else
        echo "  ✗ ${TARGET} 不存在，请先运行 bash setup_dds.sh"
    fi
    echo "--- .bashrc 中的相关行 ---"
    grep -nE "ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI" "${BASHRC}" 2>/dev/null || echo "  (无)"
    echo "==========================================="
    exit 0
fi

# ---------------- 前置检查 ----------------
[ -f "${TEMPLATE}" ] || { ERR "找不到模板 ${TEMPLATE}"; exit 1; }

if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        . /opt/ros/humble/setup.bash
    else
        WARN "未 source ROS2，部分检查会跳过"
    fi
fi

if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    WARN "rmw_cyclonedds_cpp 未安装，正在安装..."
    sudo apt-get update -y
    sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp
fi

# ---------------- 交互式补全 ----------------
echo ""
echo "可用网卡："
ip -o -4 addr show 2>/dev/null | awk '{printf "  %-12s %s\n", $2, $4}'
echo ""

if [ -z "$ROLE" ]; then
    echo "本机角色？"
    echo "  1) robot  —— 工控机（跑驱动和导航）"
    echo "  2) host   —— 我的电脑（只跑 RViz/GUI）"
    read -r -p "选 1 或 2: " choice
    ROLE=$([ "$choice" = "1" ] && echo robot || echo host)
fi

if [ -z "$IFACE" ]; then
    DEFAULT_IFACE=$(ip route 2>/dev/null | awk '/^default/ {print $5; exit}')
    read -r -p "用哪张网卡通信？[默认 ${DEFAULT_IFACE}] " IFACE
    IFACE="${IFACE:-$DEFAULT_IFACE}"
fi

if [ -z "$PEER" ]; then
    read -r -p "对端 IP 是多少？（robot 填电脑 IP / host 填工控机 IP）: " PEER
fi

if [ -z "$IFACE" ] || [ -z "$PEER" ]; then
    ERR "网卡和对端 IP 都不能为空"; exit 1
fi

# 校验网卡存在
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    ERR "网卡 ${IFACE} 不存在！可用网卡见上方列表。"; exit 1
fi

# ---------------- 渲染配置 ----------------
INFO "生成 ${TARGET}（role=${ROLE} iface=${IFACE} peer=${PEER} domain=${DOMAIN_ID}）"
sed -e "s|__IFACE__|${IFACE}|g" -e "s|__PEER_IP__|${PEER}|g" \
    "${TEMPLATE}" > "${TARGET}"

# ---------------- 写入 .bashrc ----------------
mark="# ===== DT-01 ROS2 多机通信（由 setup_dds.sh 写入） ====="
# 清除旧的标记块，避免重复追加
if grep -qF "$mark" "${BASHRC}" 2>/dev/null; then
    INFO "已存在旧配置块，先清除"
    python3 - "$BASHRC" "$mark" <<'PY'
import sys
path, mark = sys.argv[1], sys.argv[2]
lines = open(path, encoding='utf-8').read().splitlines(keepends=True)
out, skipping = [], False
for ln in lines:
    if ln.strip() == mark:
        skipping = True
        continue
    if skipping and ln.rstrip() == '# ===== end =====':
        skipping = False
        continue
    if not skipping:
        out.append(ln)
open(path, 'w', encoding='utf-8').writelines(out)
PY
fi

cat >> "${BASHRC}" <<EOF

${mark}
export ROS_DOMAIN_ID=${DOMAIN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${TARGET}
# ===== end =====
EOF

# 立即生效（当前 shell）
export ROS_DOMAIN_ID=${DOMAIN_ID}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${TARGET}

INFO "已写入 ${BASHRC}"
echo ""
echo "---------------------------------------------"
cat "${TARGET}"
echo "---------------------------------------------"
echo ""
INFO "本机配置完成。请对另一台机器执行同样的操作："
if [ "$ROLE" = "robot" ]; then
    echo "    bash setup_dds.sh --role host --peer $(hostname -I | awk '{print $1}')"
else
    echo "    bash setup_dds.sh --role robot --peer <工控机IP>"
fi
echo ""
WARN "重要：新开终端才会生效，或先执行："
echo "    export ROS_DOMAIN_ID=${DOMAIN_ID}"
echo "    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "    export CYCLONEDDS_URI=file://${TARGET}"
echo ""
INFO "验证（两边都跑起来之后，在其中一台执行）："
echo "    ros2 topic list          # 能看到对方的话题就算通了"
echo "    ros2 topic hz /scan      # 有频率说明数据真的过来了"
echo "    bash setup_dds.sh --check"
