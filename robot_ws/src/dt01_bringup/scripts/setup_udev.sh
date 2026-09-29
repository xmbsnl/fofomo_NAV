#!/usr/bin/env bash
# ============================================================
# DT-01 串口设备名固定（udev 规则生成）
#
# 为什么必须做：
#   工控机上插了 3 个 USB 串口（雷达/IMU，底盘可能也走 USB）。
#   Linux 按枚举顺序分配 ttyUSB0/1/2，重启后顺序可能变 ——
#   昨天 ttyUSB0 是底盘，今天可能是雷达。结果就是"昨天还好好的，
#   今天一开机雷达数据变成了底盘数据"，极难排查。
#
#   udev 按设备的 VID/PID/序列号生成固定符号链接：
#       /dev/dt01_chassis、/dev/dt01_lidar、/dev/dt01_imu
#   之后 launch 里写死这三个名字就永远不会错。
#
# 用法（三根线都插好之后再跑）：
#   bash setup_udev.sh              # 交互式：列出设备，你选哪个是哪个
#   bash setup_udev.sh --show       # 只列出设备和已有规则，不改任何东西
# ============================================================
set -euo pipefail

RULES="/etc/udev/rules.d/99-dt01-serial.rules"

INFO(){ echo -e "\033[32m[INFO]\033[0m $*"; }
WARN(){ echo -e "\033[33m[WARN]\033[0m $*"; }
ERR(){  echo -e "\033[31m[ERR]\033[0m $*"; }

SHOW_ONLY=0
[ "${1:-}" = "--show" ] && SHOW_ONLY=1

echo "=============== 当前串口设备 ==============="
DEVS=$(ls -1 /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null || true)
if [ -z "$DEVS" ]; then
    ERR "未检测到任何串口设备。请确认三根线都插好了再跑本脚本。"
    exit 1
fi

i=0
declare -a DEV_LIST=()
for dev in $DEVS; do
    # 只看真正存在的字符设备，跳过 ttyS 里大量不存在的占位
    [ -c "$dev" ] || continue
    i=$((i + 1))
    DEV_LIST+=("$dev")
    echo ""
    echo "  [$i] $dev"
    udevadm info --query=property --name="$dev" 2>/dev/null \
        | grep -E "^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_VENDOR=|ID_MODEL=)" \
        | sed 's/^/      /' \
        || echo "      (板载串口，无 USB 属性，属正常)"
done
echo ""
echo "==========================================="

if [ $SHOW_ONLY -eq 1 ]; then
    echo ""
    echo "已有 udev 规则（$RULES）："
    if [ -f "$RULES" ]; then
        cat "$RULES"
    else
        echo "  (无，运行 bash setup_udev.sh 生成)"
    fi
    echo ""
    echo "当前符号链接："
    ls -l /dev/dt01_* 2>/dev/null || echo "  (无)"
    exit 0
fi

if [ -f "$RULES" ]; then
    WARN "已有规则文件 $RULES，将先备份为 ${RULES}.bak"
    sudo cp "$RULES" "${RULES}.bak"
fi

TOTAL=${#DEV_LIST[@]}
echo ""
INFO "下面给每个串口分配角色。输入编号，或 0 跳过。"
echo "   角色：chassis=底盘  lidar=激光雷达  imu=IMU"
echo ""

declare -A ROLE_OF
for role in chassis lidar imu; do
    while true; do
        read -r -p "哪个是 ${role}？(1-${TOTAL}，0=没有/跳过): " n
        if [ "$n" = "0" ]; then
            WARN "跳过 ${role}"
            break
        fi
        if [[ "$n" =~ ^[0-9]+$ ]] && [ "$n" -ge 1 ] && [ "$n" -le "$TOTAL" ]; then
            ROLE_OF[$role]="${DEV_LIST[$((n - 1))]}"
            INFO "${role} -> ${ROLE_OF[$role]}"
            break
        fi
        ERR "请输入 0 到 ${TOTAL} 之间的数字"
    done
done

# ---------------- 生成规则 ----------------
TMP=$(mktemp)
cat > "$TMP" <<'EOF'
# ============================================================
# DT-01 串口设备固定名（由 setup_udev.sh 生成）
# 依据：USB 设备的 VID/PID/序列号，或板载串口的内核名
# 重新生成：bash setup_udev.sh
# ============================================================
EOF

gen_usb_rule() {
    local dev="$1" link="$2"
    local vid pid serial
    vid=$(udevadm info --query=property --name="$dev" 2>/dev/null \
          | sed -n 's/^ID_VENDOR_ID=//p')
    pid=$(udevadm info --query=property --name="$dev" 2>/dev/null \
          | sed -n 's/^ID_MODEL_ID=//p')
    serial=$(udevadm info --query=property --name="$dev" 2>/dev/null \
             | sed -n 's/^ID_SERIAL_SHORT=//p')
    if [ -n "$vid" ] && [ -n "$serial" ]; then
        # 有序列号：最可靠，同型号多个设备也不会混淆
        printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", ATTRS{serial}=="%s", SYMLINK+="dt01_%s", MODE="0666"\n' \
            "$vid" "$pid" "$serial" "$link"
    elif [ -n "$vid" ]; then
        # 无序列号：只能靠 VID/PID，同型号多个设备会冲突，给出警告
        WARN "${dev} 没有序列号，只能按 VID/PID 匹配；若有两个同型号设备会认错"
        printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", SYMLINK+="dt01_%s", MODE="0666"\n' \
            "$vid" "$pid" "$link"
    else
        # 板载串口（ttyS*）：按内核名固定
        printf 'SUBSYSTEM=="tty", KERNEL=="%s", SYMLINK+="dt01_%s", MODE="0666"\n' \
            "$(basename "$dev")" "$link"
    fi
}

for role in chassis lidar imu; do
    dev="${ROLE_OF[$role]:-}"
    [ -n "$dev" ] || continue
    gen_usb_rule "$dev" "$role" >> "$TMP"
done

echo ""
INFO "将写入 ${RULES}："
echo "---------------------------------------------"
cat "$TMP"
echo "---------------------------------------------"

sudo cp "$TMP" "$RULES"
rm -f "$TMP"
sudo chmod 644 "$RULES"

INFO "重载 udev 规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger
sleep 2

echo ""
INFO "生成的固定设备名："
ls -l /dev/dt01_* 2>/dev/null || WARN "未看到 /dev/dt01_*，请重新插拔 USB 线后再 ls -l /dev/dt01_*"

echo ""
WARN "如果符号链接没出现，拔掉 USB 再插一次（udev trigger 对已插设备有时不生效）"
echo ""
INFO "之后启动机器人就可以写死设备名了："
echo "    ros2 launch dt01_bringup dt01_robot.launch.py \\"
echo "        chassis_port:=/dev/dt01_chassis \\"
echo "        lidar_port:=/dev/dt01_lidar \\"
echo "        imu_port:=/dev/dt01_imu"
