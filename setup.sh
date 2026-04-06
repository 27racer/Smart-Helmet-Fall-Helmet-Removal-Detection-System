#!/bin/bash
# ═══════════════════════════════════════════════════════════
#  Smart Helmet — Raspberry Pi 5 Setup Script
#  Run once:  chmod +x setup.sh && sudo ./setup.sh
# ═══════════════════════════════════════════════════════════

set -e
echo "╔══════════════════════════════════════════╗"
echo "║  Smart Helmet — Setup Script             ║"
echo "╚══════════════════════════════════════════╝"

# 1. System packages
echo ""
echo "[1/4] Installing system dependencies..."
apt-get update
apt-get install -y python3-pip python3-venv i2c-tools libgpiod3 python3-libgpiod

# 2. Enable I2C (if not already enabled)
echo ""
echo "[2/4] Enabling I2C interface..."
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "dtparam=i2c_arm=on" >> /boot/firmware/config.txt
    echo "  → I2C enabled in config.txt (reboot required)"
else
    echo "  → I2C already enabled"
fi

# 3. Create virtual environment & install Python packages
echo ""
echo "[3/4] Setting up Python virtual environment..."
VENV_DIR="/home/$(logname)/smart_helmet/venv"
if [ ! -d "$VENV_DIR" ]; then
    sudo -u "$(logname)" python3 -m venv "$VENV_DIR"
fi
sudo -u "$(logname)" "$VENV_DIR/bin/pip" install --upgrade pip
sudo -u "$(logname)" "$VENV_DIR/bin/pip" install -r /home/$(logname)/Smart-Helmet-Fall-Helmet-Removal-Detection-System/requirements.txt

# 4. Verify I2C devices
echo ""
echo "[4/4] Scanning I2C bus..."
echo "  Expected addresses:"
echo "    0x23 — BH1750 (light)"
echo "    0x29 — VL53L0X (ToF distance)"
echo "    0x3C — SSD1306 (OLED)"
echo "    0x68 — MPU6050 (IMU)"
echo ""
i2cdetect -y 1 || echo "  ⚠ I2C scan failed — you may need to reboot first."

echo ""
echo "═══════════════════════════════════════════"
echo "  Setup complete!"
echo ""
echo "  If this is the first time enabling I2C:"
echo "    sudo reboot"
echo ""
echo "  To run the smart helmet:"
echo "    cd ~/smart_helmet"
echo "    source venv/bin/activate"
echo "    python smart_helmet.py"
echo "═══════════════════════════════════════════"
