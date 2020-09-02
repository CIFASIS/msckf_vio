help:
	@echo "help   -- print this help"
	@echo "shell  -- open shell in container"
	@echo "image  -- build Docker image"


image:
	docker build -t "msckf_vio:ros_kinetic" .
shell:
	docker run -it --net=host msckf_vio:ros_kinetic bash

.PHONY: help shell image

