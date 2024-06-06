#!/bin/bash
# ------------------------------------------------------------------------------
# Description:
#	Utility script to make first Espressif board programming, in particular:
#		1- 	generate RSA key pair for device.
#		2-	get device MAC address.
#		3-	store both key pair and MAC address into report folder.
#		4-	flash firmware into device and make first power up test.
#		5-	send MAC and key pair to Datasmart endpoint and wait for 
#			registration.
#		6-	check if device successfully register to GIOTC.
#		7-	backup credential folder to server. [already git synched]
# 
# Requirements:
#	esptool instatlled.
#		reference to "https://github.com/espressif/esptool".
#		to install run  "pip install esptool".
#
# Parameters:
#	<none>
#
# Usage:
#	. giotc_prog_n_reg.sh
# ------------------------------------------------------------------------------
echo "hello world."


# Script functional variables.
USAGE=". progutil.sh"
EDNPOINT_DATASMART=https://smart-tech-lite-services.datasmart.cloud/api/Devices/Registration/xYoN99hk63jrfYTzzYofNq1jx2xobcg7

# Get MAC address.
MAC=`esptool.py read_mac | grep -m 1 "MAC" | cut -d' ' -f2`
MACSTR="${MAC//:/}"
PATHDIR="$MACSTR"
echo $PATHDIR

# Check if folder already exists, if so current device has been already registered
# and to reflash has to be used relative rsa_private.pem. They are stored giotc folder.
if [ -d "$PATHDIR" ]; then
 	echo "${PATHDIR} already exist, copy rsa_private.pem to firmware folder."
	# Copy privte key to cert folder.
	cp roots.pem ../main/certs/roots.pem
	cp $PATHDIR/rsa_private.pem ../main/certs/rsa_private.pem
	return;
fi


# Create credential folder.
mkdir -p $PATHDIR


# Generate rsa key pair.
openssl genpkey -algorithm RSA -out $PATHDIR/rsa_private.pem -pkeyopt rsa_keygen_bits:2048
openssl rsa -in $PATHDIR/rsa_private.pem -pubout -out $PATHDIR/rsa_public.pem
PUBRSA=`cat $PATHDIR/rsa_public.pem`
PUBRSA=`sed -E ':a;N;$!ba;s/\r{0,1}\n/\\\n/g' $PATHDIR/rsa_public.pem`


# Copy privte key to cert/xphkey folder.
#cp $PATHDIR/rsa_private.pem main/certs/rsa_private.pem


# Program device with esptool
echo "Program device."
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x15000 build/ota_data_initial.bin 0x1000 build/bootloader/bootloader.bin 0x11000 build/phy_init_data.bin 0x20000 build/esp32magniflex-0.0.bin 0x8000 build/myptable.bin


# Check output.
RET = `stty -F /dev/ttyUSB0 raw speed 115200` # Port setting.


# Check wifi.
echo "Get device function."
cat < /dev/ttyUSB0 | 
while IFS= read -r line
do
	if echo "$line" | grep "wifi"
	then
		break;
	fi
done
	

# Write mac and pubkey to json data file.
#echo "{\"mac\":\"$MAC\",\"pubKey\":\"$PUBRSA\"}"
printf "{\"mac\":\"$MAC\",\"pubKey\":\"$PUBRSA\"}" > $PATHDIR/credential.json


# Send json data file to datasmart endpoint.
RETHTTP=`curl -H "Accept: application/json" -H "Content-type: application/json" -X POST -d "{\"mac\":\"$MAC\",\"pubKey\":\"$PUBRSA\"}" $EDNPOINT_DATASMART`


# following is the correct answer.
# ------------------------------------------------------------------------------
# {"item":{"result":true},"code":0,"message":"No Error","description":""}
# ------------------------------------------------------------------------------
echo $RETHTTP
if echo "$RETHTTP" | grep 'true'; then
  echo "Device registration succeeded."
else 
  echo "Device registration failed."
fi

# Copy new .pem files into build path.
cp roots.pem ../main/certs/roots.pem
cp $PATHDIR/rsa_private.pem ../main/certs/rsa_private.pem




