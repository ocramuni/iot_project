#!/bin/bash

for c in telegraf mosquitto influxdb grafana
do
	podman stop $c
	podman rm $c
done

podman pod rm iot
podman volume prune -f
