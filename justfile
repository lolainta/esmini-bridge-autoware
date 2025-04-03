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
