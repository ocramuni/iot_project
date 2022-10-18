#!/bin/bash

PROJECT_PATH="$HOME/Projects/pwr_iot/docker"

podman pod create --name iot -p=1883:1883,8086:8086,8125:8125/udp,3000:3000

podman run -d --pod=iot \
      --name=influxdb \
      -v $PROJECT_PATH/influxdb/data:/var/lib/influxdb2:Z \
      -v $PROJECT_PATH/influxdb/config:/etc/influxdb2:Z \
      -e TZ=Europe/Rome \
      -e DOCKER_INFLUXDB_INIT_MODE=setup \
      -e DOCKER_INFLUXDB_INIT_USERNAME=user \
      -e DOCKER_INFLUXDB_INIT_PASSWORD=password \
      -e DOCKER_INFLUXDB_INIT_ORG=org \
      -e DOCKER_INFLUXDB_INIT_BUCKET=telegraf \
      -e DOCKER_INFLUXDB_INIT_ADMIN_TOKEN='jcTCQ5PRSjOtj5ur0TkGc9y93aRYbO9yX7MN8l7D9_dWxc4rmDSnp5a7dSUNzhD3wAdSzVdZu2qCUsV5iaIwQg==' \
      influxdb

podman run -d --pod=iot --name mosquitto -e TZ=Europe/Rome -v $PROJECT_PATH/mosquitto/:/mosquitto/:Z  eclipse-mosquitto

podman run -d --pod=iot --name telegraf -u "telegraf" -e TZ=Europe/Rome -v $PROJECT_PATH/telegraf/telegraf.conf:/etc/telegraf/telegraf.conf:ro telegraf

podman run -d --pod=iot --name=grafana \
	--volume "$PROJECT_PATH/grafana/data:/var/lib/grafana:Z" \
        -e TZ=Europe/Rome \
	grafana/grafana-oss

