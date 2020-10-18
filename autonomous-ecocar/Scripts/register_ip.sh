#!/bin/bash
ip_address="$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')"
echo "Got:$ip_address"
curl "https://dtucar.com/a/ip_sync.php?ident=20170665&ip=$ip_address"
