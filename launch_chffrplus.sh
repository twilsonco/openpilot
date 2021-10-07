#!/usr/bin/bash

if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi

file="/data/no_ota_updates"


source "$BASEDIR/launch_env.sh"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

function two_init {
  # Restrict Android and other system processes to the first two cores
  echo 0-1 > /dev/cpuset/background/cpus
  echo 0-1 > /dev/cpuset/system-background/cpus
  echo 0-1 > /dev/cpuset/foreground/cpus
  echo 0-1 > /dev/cpuset/foreground/boost/cpus
  echo 0-1 > /dev/cpuset/android/cpus

  # openpilot gets all the cores
  echo 0-3 > /dev/cpuset/app/cpus

  # set up governors
  # +50mW offroad, +500mW onroad for 30% more RAM bandwidth
  echo "performance" > /sys/class/devfreq/soc:qcom,cpubw/governor
  echo 1056000 > /sys/class/devfreq/soc:qcom,m4m/max_freq
  echo "performance" > /sys/class/devfreq/soc:qcom,m4m/governor

  # unclear if these help, but they don't seem to hurt
  echo "performance" > /sys/class/devfreq/soc:qcom,memlat-cpu0/governor
  echo "performance" > /sys/class/devfreq/soc:qcom,memlat-cpu2/governor

  # GPU
  echo "performance" > /sys/class/devfreq/b00000.qcom,kgsl-3d0/governor

  # /sys/class/devfreq/soc:qcom,mincpubw is the only one left at "powersave"
  # it seems to gain nothing but a wasted 500mW

  # Collect RIL and other possibly long-running I/O interrupts onto CPU 1
  echo 1 > /proc/irq/78/smp_affinity_list # qcom,smd-modem (LTE radio)
  echo 1 > /proc/irq/33/smp_affinity_list # ufshcd (flash storage)
  echo 1 > /proc/irq/35/smp_affinity_list # wifi (wlan_pci)
  echo 1 > /proc/irq/6/smp_affinity_list  # MDSS

  # USB traffic needs realtime handling on cpu 3
  [ -d "/proc/irq/733" ] && echo 3 > /proc/irq/733/smp_affinity_list # USB for LeEco
  [ -d "/proc/irq/736" ] && echo 3 > /proc/irq/736/smp_affinity_list # USB for OP3T

  
  if ! [ -f "$file" ]; then
    # Check for NEOS update
    if [ $(< /VERSION) != "$REQUIRED_NEOS_VERSION" ]; then
      if [ -f "$DIR/scripts/continue.sh" ]; then
        cp "$DIR/scripts/continue.sh" "/data/data/com.termux/files/continue.sh"
      fi

      if [ ! -f "$BASEDIR/prebuilt" ]; then
        # Clean old build products, but preserve the scons cache
        cd $DIR
        scons --clean
        git clean -xdf
        git submodule foreach --recursive git clean -xdf
      fi
  
      "$DIR/installer/updater/updater" "file://$DIR/installer/updater/update.json"
    fi
  fi

  # One-time fix for a subset of OP3T with gyro orientation offsets.
  # Remove and regenerate qcom sensor registry. Only done on OP3T mainboards.
  # Performed exactly once. The old registry is preserved just-in-case, and
  # doubles as a flag denoting we've already done the reset.
  if ! $(grep -q "letv" /proc/cmdline) && [ ! -f "/persist/comma/op3t-sns-reg-backup" ]; then
    echo "Performing OP3T sensor registry reset"
    mv /persist/sensors/sns.reg /persist/comma/op3t-sns-reg-backup &&
      rm -f /persist/sensors/sensors_settings /persist/sensors/error_log /persist/sensors/gyro_sensitity_cal &&
      echo "restart" > /sys/kernel/debug/msm_subsys/slpi &&
      sleep 5  # Give Android sensor subsystem a moment to recover
  fi
}

function tici_init {
  sudo su -c 'echo "performance" > /sys/class/devfreq/soc:qcom,memlat-cpu0/governor'
  sudo su -c 'echo "performance" > /sys/class/devfreq/soc:qcom,memlat-cpu4/governor'

  # set success flag for current boot slot
  sudo abctl --set_success

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    # Get number of slot to switch to
    CUR_SLOT=$(abctl --boot_slot)
    if [[ "$CUR_SLOT" == "_a" ]]; then
      OTHER_SLOT="_b"
      OTHER_SLOT_NUMBER="1"
    else
      OTHER_SLOT="_a"
      OTHER_SLOT_NUMBER="0"
    fi
    echo "Cur slot $CUR_SLOT, target $OTHER_SLOT"

    # Get expected hashes from manifest
    MANIFEST="$DIR/selfdrive/hardware/tici/agnos.json"
    SYSTEM_HASH_EXPECTED=$(jq -r ".[] | select(.name == \"system\") | .hash_raw" $MANIFEST)
    SYSTEM_SIZE=$(jq -r ".[] | select(.name == \"system\") | .size" $MANIFEST)
    BOOT_HASH_EXPECTED=$(jq -r ".[] | select(.name == \"boot\") | .hash_raw" $MANIFEST)
    BOOT_SIZE=$(jq -r ".[] | select(.name == \"boot\") | .size" $MANIFEST)
    echo "Expected hashes:"
    echo "System: $SYSTEM_HASH_EXPECTED"
    echo "Boot: $BOOT_HASH_EXPECTED"

    # Read hashes from alternate partitions, should already be flashed by updated
    SYSTEM_HASH=$(dd if="/dev/disk/by-partlabel/system$OTHER_SLOT" bs=1 skip="$SYSTEM_SIZE" count=64 2>/dev/null)
    BOOT_HASH=$(dd if="/dev/disk/by-partlabel/boot$OTHER_SLOT" bs=1 skip="$BOOT_SIZE" count=64 2>/dev/null)
    echo "Found hashes:"
    echo "System: $SYSTEM_HASH"
    echo "Boot: $BOOT_HASH"

    if [[ "$SYSTEM_HASH" == "$SYSTEM_HASH_EXPECTED" && "$BOOT_HASH" == "$BOOT_HASH_EXPECTED" ]]; then
      echo "Swapping active slot to $OTHER_SLOT_NUMBER"

      # Clean hashes before swapping to prevent looping
      dd if=/dev/zero of="/dev/disk/by-partlabel/system$OTHER_SLOT" bs=1 seek="$SYSTEM_SIZE" count=64
      dd if=/dev/zero of="/dev/disk/by-partlabel/boot$OTHER_SLOT" bs=1 seek="$BOOT_SIZE" count=64
      sync

      abctl --set_active "$OTHER_SLOT_NUMBER"

      sleep 1
      sudo reboot
    else
      echo "Hash mismatch, downloading agnos"
      if $DIR/selfdrive/hardware/tici/agnos.py $MANIFEST; then
        echo "Download done, swapping active slot to $OTHER_SLOT_NUMBER"

        # Clean hashes before swapping to prevent looping
        dd if=/dev/zero of="/dev/disk/by-partlabel/system$OTHER_SLOT" bs=1 seek="$SYSTEM_SIZE" count=64
        dd if=/dev/zero of="/dev/disk/by-partlabel/boot$OTHER_SLOT" bs=1 seek="$BOOT_SIZE" count=64
        sync

        abctl --set_active "$OTHER_SLOT_NUMBER"
      fi

      sleep 1
      sudo reboot
    fi
  fi
}

function launch {
  # Wifi scan
  wpa_cli IFNAME=wlan0 SCAN

  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The BASEDIR init file has to exist, with a newer modtime than anything in
  #    the BASEDIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if ! [ -f "$file" ]; then
    if [ "$(git rev-parse HEAD)" != "$(git rev-parse @{u})" ]; then
      git reset --hard @{u} &&
      git clean -xdf &&
      exec "${BASH_SOURCE[0]}"
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # hardware specific init
  if [ -f /EON ]; then
    two_init
  fi

  if [ -f /TICI ]; then
    tici_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd selfdrive
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
