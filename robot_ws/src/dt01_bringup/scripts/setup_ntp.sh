#!/usr/bin/env bash
# ============================================================
# DT-01 时间同步（chrony）—— 上真机必做，但最容易忘
#
# 为什么要同步：
#   两台机器各自跑自己的系统时钟，差几秒是常态。跨机 TF 的时间戳来自
#   不同时钟，tf2 会疯狂刷 "TF_OLD_DATA" / "Extrapolation into the past"，
#   表现是 RViz 里机器人图标乱跳、costmap 报变换失败、Nav2 拒绝规划。
#
# 为什么用工控机当时间源：
#   RK3588 没有 RTC 电池，断电后时间会回到出厂值。但它开机后会一直运行，
#   所以让工控机以 local stratum 10 兜底对外授时，上位机同步到它，
#   两边相对时间就是一致的（绝对时间准不准对导航来说不重要）。
#
# 用法：
#   工控机上：bash setup_ntp.sh --server [网段]     # 不传则用下面的 DEFAULT_SUBNET
#   电脑上  ：bash setup_ntp.sh --client <工控机IP>
#   体检    ：bash setup_ntp.sh --check
#
# ⚠️ 网段必须和「工控机↔上位机」通信所用网段一致，不是雷达网段。
#    换场地/换路由器后记得改 DEFAULT_SUBNET 或显式传参，否则 allow 网段
#    对不上，客户端 chronyc tracking 会一直是 Reference ID : 00000000（没同步上）。
# ============================================================
set -euo pipefail

MODE=""
SUBNET="192.168.2.0/24"
SERVER_IP=""

INFO(){ echo -e "\033[32m[INFO]\033[0m $*"; }
WARN(){ echo -e "\033[33m[WARN]\033[0m $*"; }
ERR(){  echo -e "\033[31m[ERR]\033[0m $*"; }

usage() {
    cat <<EOF
用法：
  bash setup_ntp.sh --server [网段]      # 工控机：作为时间源，允许网段内机器同步
  bash setup_ntp.sh --client <工控机IP>  # 电脑：同步到工控机
  bash setup_ntp.sh --check              # 查看当前同步状态
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --server) MODE=server; [ $# -gt 1 ] && [[ "$2" != -* ]] && { SUBNET="$2"; shift; }; shift;;
        --client) MODE=client; SERVER_IP="${2:-}"; shift 2;;
        --check)  MODE=check;  shift;;
        -h|--help) usage; exit 0;;
        *) ERR "未知参数: $1"; usage; exit 1;;
    esac
done

# ---------------- 体检 ----------------
if [ "$MODE" = "check" ]; then
    echo "=============== 时间同步体检 ==============="
    echo "--- 本机时间 ---"
    date '+  %Y-%m-%d %H:%M:%S %Z'
    echo "--- 时间源 ---"
    if command -v chronyc >/dev/null 2>&1; then
        chronyc sources -v 2>/dev/null | sed 's/^/  /' || echo "  (chronyd 未运行)"
        echo "--- 同步状态 ---"
        chronyc tracking 2>/dev/null | grep -E "Reference ID|Stratum|Last offset|Leap status" \
            | sed 's/^/  /' || echo "  (chronyd 未运行)"
    else
        echo "  ✗ chrony 未安装。执行：sudo apt install -y chrony"
    fi
    echo "--- 与对端的时间差（需手工填 IP 对比）---"
    echo "  在另一台机器上执行 date 对比即可，误差应 < 1s"
    echo "=========================================="
    exit 0
fi

[ -n "$MODE" ] || { ERR "请指定 --server / --client / --check"; usage; exit 1; }

# ---------------- 安装 chrony ----------------
if ! command -v chronyc >/dev/null 2>&1; then
    INFO "安装 chrony..."
    sudo apt-get update -y
    sudo apt-get install -y chrony
fi

CHRONY_CONF="/etc/chrony/chrony.conf"
[ -f "$CHRONY_CONF" ] || CHRONY_CONF="/etc/chrony.conf"
[ -f "$CHRONY_CONF" ] || { ERR "找不到 chrony 配置文件"; exit 1; }

if [ "$MODE" = "server" ]; then
    INFO "配置本机（工控机）为时间源，允许 ${SUBNET} 同步"
    if ! grep -qF "allow ${SUBNET}" "$CHRONY_CONF"; then
        sudo cp "$CHRONY_CONF" "${CHRONY_CONF}.bak.dt01"
        sudo tee -a "$CHRONY_CONF" >/dev/null <<EOF

# ===== DT-01 时间源配置（由 setup_ntp.sh 写入）=====
# 即使本机没有外网、没有可靠时间源，也继续对外授时。
# RK3588 无 RTC 电池，断电重启后时间会回退，但导航只关心两台机器的
# 相对一致性，所以这里强制以本机为权威。
allow ${SUBNET}
local stratum 10
EOF
    else
        INFO "配置已存在，跳过写入"
    fi
    sudo systemctl restart chrony 2>/dev/null || sudo systemctl restart chronyd
    sudo systemctl enable chrony 2>/dev/null || sudo systemctl enable chronyd
    sleep 2
    INFO "工控机时间源已就绪：$(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    INFO "接下来在你的电脑上执行："
    echo "    bash setup_ntp.sh --client $(hostname -I | awk '{print $1}')"

else
    [ -n "$SERVER_IP" ] || { ERR "--client 需要指定工控机 IP"; usage; exit 1; }
    INFO "配置本机同步到 ${SERVER_IP}"
    if ! grep -qF "server ${SERVER_IP} iburst" "$CHRONY_CONF"; then
        sudo cp "$CHRONY_CONF" "${CHRONY_CONF}.bak.dt01"
        # 用 makestep 允许首次大步修正；否则时钟差太大时 chrony 只会慢慢爬，
        # 可能要几十分钟才对齐，期间 TF 一直在报错。
        sudo sed -i "s/^makestep .*/makestep 1.0 3/" "$CHRONY_CONF" 2>/dev/null || true
        sudo tee -a "$CHRONY_CONF" >/dev/null <<EOF

# ===== DT-01 客户端配置（由 setup_ntp.sh 写入）=====
server ${SERVER_IP} iburst prefer
makestep 1.0 3
EOF
    else
        INFO "配置已存在，跳过写入"
    fi
    sudo systemctl restart chrony 2>/dev/null || sudo systemctl restart chronyd
    sudo systemctl enable chrony 2>/dev/null || sudo systemctl enable chronyd
    sleep 3
    INFO "同步状态："
    chronyc tracking 2>/dev/null | grep -E "Reference ID|Stratum|Last offset" | sed 's/^/  /'
    echo ""
    WARN "若 Last offset 仍是几百毫秒，等 10 秒后再跑一次："
    echo "    bash setup_ntp.sh --check"
fi
