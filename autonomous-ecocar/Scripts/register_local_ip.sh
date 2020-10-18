#!/bin/bash
ip_address="192.168.43.242"
echo "Got:$ip_address"
curl "https://dtucar.com/a/ip_sync.php?ident=20170665&ip=$ip_address"
