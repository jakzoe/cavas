#!/bin/bash

get_tty_path() {
  local VENDOR_ID="$1"
  local PRODUCT_ID="$2"
  local SERIAL_ID="$3"

  DEVICE_PATH=$(for device in /dev/ttyUSB*; do
    if udevadm info -a -n "$device" | grep -q "ATTRS{idVendor}==\"$VENDOR_ID\"" &&
       udevadm info -a -n "$device" | grep -q "ATTRS{idProduct}==\"$PRODUCT_ID\""; then

      if [ -n "$SERIAL_ID" ]; then
        if udevadm info -a -n "$device" | grep -q "ATTRS{serial}==\"$SERIAL_ID\""; then
          echo "$device"
        fi
      else
        echo "$device"
      fi
    fi
  done)

  if [ -n "$DEVICE_PATH" ]; then
    echo "$DEVICE_PATH"
  else
    echo ""
  fi
}


run_docker_with_device() {
  local input="$1"
  local bus=$(echo "$input" | awk '{print $2}' | sed 's/://')
  local device=$(echo "$input" | awk '{print $4}' | sed 's/://')
  local spec_path="/dev/bus/usb/$bus/$device"
  # serial_path=$(get_tty_path "1a86" "7523") # Arduino (not used anymore)
  serial_path=$(get_tty_path "0403" "6001" "A5069RR4") # FT232 Serial
  nkt_path=$(get_tty_path "10c4" "ea60")
  # the LTB uses an FT232 too, it seems, thus having the same IDs, Thus checking for iSerial too
  # (lsusb -v -d 0403:6001 | grep iSerial)
  ltb_path=$(get_tty_path "0403" "6001" "FTDD2M08")

   local devices=""

  if [ -n "$spec_path" ]; then
    devices+="--device=$spec_path "
  else
    spec_path="none"
  fi

  if [ -n "$serial_path" ]; then
    devices+="--device=$serial_path "
  else
    serial_path="none"
  fi

  if [ -n "$nkt_path" ]; then
    devices+="--device=$nkt_path "
  else
    nkt_path="none"
  fi

  if [ -n "$ltb_path" ]; then
    devices+="--device=$ltb_path "
  else
    ltb_path="none"
  fi

  if [[ ! -c "$spec_path" ]]; then
    echo "Could not find the spectrometer. Exiiting."
    return 1
  fi

  echo "Using serial: $serial_path"
  echo "Using NKT: $nkt_path"
  echo "Using LTB: $ltb_path"
  echo "Using spectrometer: $spec_path"
  echo

  # port for attaching debugger
  docker run -v /home/user/jufo/jufo2024/cavas:/root/jufo \
    -e "DISPLAY=$DISPLAY" \
    --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix \
    --device=/dev/dri:/dev/dri \
    --rm \
    -p 5678:5678 \
    $devices \
    laserdocker "$serial_path" "$nkt_path" "$ltb_path"
    
  return $?
}

# udevadm info -n /dev/ttyUSB0 or udevadm info -q property --property=ID_VENDOR_ID --value -n /dev/ttyUSB0 , productid is ID_MODEL_ID

# in my case, to prevent the PC from shutting down after a certain period of inactivity
pkill hypridle
# IDs of the spectrometer and the IDs of the device that is created after init of the spectrometer
run_docker_with_device "$(lsusb  -d 04b4:8613)"
run_docker_with_device "$(lsusb  -d 0bd7:a012)"

# Arduino. Try this, and when it fails, do it without adding it (so that I can select whether the Arduino should stil be availible to the Arduino IDE or not)
# probably need some udev hooks in Dockerfile as well then.
# As long as I am root in the Dockerfile/the devices are 666 anyway (nouser/nogroup problem), this does not matter anyway though, does not make a difference.
# run_docker_with_device "$(lsusb  -d 1a86:7523)"
