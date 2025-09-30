build:
  docker compose build

up: build
  docker compose up -d

down:
  docker compose down

exec-autoware:
  docker exec -it -e DISPLAY eba-autoware bash

exec-esmini:
  docker exec -it -e DISPLAY eba-esmini bash

run-autoware:
  docker exec -it eba-autoware just launch hct_6

run-esmini:
  docker exec -it eba-esmini just deps build
  docker exec -it eba-esmini just default /resources/xosc/chengyu/para_test01FR-ZZ_02FR-CI_2.xosc
